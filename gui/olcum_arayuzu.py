"""
Hafta-01 UART olcum arayuzu.

    py gui/olcum_arayuzu.py            # normal kullanim
    py gui/olcum_arayuzu.py --selftest # donanimsiz duman testi

Arayuz yalnizca gosterir ve kaydeder; olcume karismaz:
  * Tum zamanlar MCU'da (TIM2, 1 us) alinir; PC saati hicbir hesapta kullanilmaz.
  * Seri porta yalnizca kayit DISINDA yazar ("S<n>" senaryo, "?" durum, "B<baud>" hiz).

UART hizi degisimi: "B<baud>" -> MCU eski hizda "BAUD,<yeni>,<eski>" der ve hizini degistirir ->
PC de yeni hiza gecip "?" gonderir -> SCN gelirse tamam. Gelmezse MCU 3 sn sonra, PC de
kendiliginden eski hiza doner.
"""
from __future__ import annotations

import argparse
import os
import queue
import sys
import tempfile
import threading
import tkinter as tk
from tkinter import messagebox, ttk

import matplotlib

matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg  # noqa: E402
from matplotlib.figure import Figure  # noqa: E402

import core  # noqa: E402

try:
    import serial
    import serial.tools.list_ports
except ImportError:  # selftest seri port olmadan da calissin
    serial = None

BAUD_ACK_MS = 1500        # "B<baud>" sonrasi BAUD onayi icin bekleme
BAUD_CONFIRM_MS = 2000    # yeni hizda SCN icin bekleme
MCU_REVERT_MS = 3300      # MCU'nun eski hiza donmesi (firmware: 3000 ms) + pay
PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_OUT = os.path.join(PROJECT_DIR, "measurement")

MAX_TREE_ROWS = 300
MAX_RAW_LINES = 600
REDRAW_MS = 400

STAGE_COLORS = ("#4C78A8", "#72B7B2", "#F58518", "#E45756")
LOSS_STYLE = {
    core.DROPI: ("v", "#9467BD", "Drop (buttonQ)"),
    core.DROPQ: ("v", "#8C564B", "Drop (txQ)"),
    core.TXERR: ("s", "#D62728", "TX hata"),
    core.TMO: ("s", "#FF7F0E", "Timeout"),
    core.LOST: ("x", "#7F7F7F", "Kayıt kaybı (MCU)"),
    core.LOST_PC: ("x", "#000000", "Kayıt kaybı (PC)"),
}


# ----------------------------------------------------------------------------- seri port
class SerialLink:
    """Ayri thread'de satir okur; satirlari GUI thread'ine kuyrukla iletir."""

    def __init__(self, out_q: "queue.Queue[tuple[str, str]]"):
        self.q = out_q
        self.ser = None
        self.thread = None
        self.running = False

    @staticmethod
    def ports():
        if serial is None:
            return []
        return [(p.device, p.description) for p in serial.tools.list_ports.comports()]

    def open(self, port: str, baud: int) -> None:
        self.ser = serial.Serial(port, baud, timeout=0.1)
        self.ser.reset_input_buffer()
        self.running = True
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self) -> None:
        buf = b""
        while self.running:
            try:
                data = self.ser.read(self.ser.in_waiting or 1)
            except Exception as exc:  # kablo cekildi vb.
                self.q.put(("error", str(exc)))
                break
            if not data:
                continue
            buf += data
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                self.q.put(("line", line.decode("ascii", errors="replace").rstrip("\r")))

    def set_baud(self, baud: int) -> None:
        if self.ser is not None:
            self.ser.baudrate = baud          # pyserial acik portta hizi degistirebilir
            self.ser.reset_input_buffer()

    def write(self, text: str) -> None:
        if self.ser is not None:
            self.ser.write(text.encode("ascii"))

    def close(self) -> None:
        self.running = False
        if self.thread is not None:
            self.thread.join(timeout=1.0)
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None
        self.thread = None

    @property
    def is_open(self) -> bool:
        return self.ser is not None


# ----------------------------------------------------------------------------- uygulama
class App:
    def __init__(self, root: tk.Tk, out_dir: str):
        self.root = root
        self.out_dir = out_dir
        core.ensure_measurement_folder(out_dir)

        self.q: "queue.Queue[tuple[str, str]]" = queue.Queue()
        self.link = SerialLink(self.q)

        self.mcu_scn = None              # MCU'nun SCN satiriyla bildirdigi senaryo
        self.mcu_baud = None             # MCU'nun SCN satiriyla bildirdigi UART hizi
        self.link_baud = core.DEFAULT_BAUD
        self.baud_sw = None              # suren hiz degisimi: {"new", "old", "phase", "t0"}
        self.scn_seen = False
        self.next_expected_id = None     # SCN.next_id veya son REC.id + 1
        self.exp = None                  # kayit suren deney
        self.last_exp = None             # son biten deney (grafik icin)
        self.disk = core.load_all(out_dir)
        self.parse_errors = 0
        self.tel_count = {s: 0 for s in range(core.SCENARIO_COUNT)}
        self.tel_exec_max = 0
        self.tel_exec_sum = 0
        self.tel_exec_n = 0
        self.dirty = True
        self.flash_job = None
        self.confirm = messagebox.askyesno   # selftest'te degistirilir

        root.title("Hafta-01 - UART Ölçüm Arayüzü")
        root.geometry("1380x860")
        root.minsize(1100, 700)
        self._build_ui()
        self._refresh_ports()
        self._refresh_summary_table()
        self._update_buttons()
        self.root.after(50, self._poll)
        self.root.after(REDRAW_MS, self._redraw_tick)
        root.protocol("WM_DELETE_WINDOW", self._on_close)

    # ------------------------------------------------------------------ arayuz
    def _build_ui(self) -> None:
        style = ttk.Style()
        style.configure("Big.TLabel", font=("Segoe UI", 18, "bold"))
        style.configure("Mid.TLabel", font=("Segoe UI", 12, "bold"))
        style.configure("Hint.TLabel", foreground="#666666")

        # --- ust bar: port
        top = ttk.Frame(self.root, padding=(10, 8))
        top.pack(side=tk.TOP, fill=tk.X)
        ttk.Label(top, text="Port:").pack(side=tk.LEFT)
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(top, textvariable=self.port_var, width=38, state="readonly")
        self.port_combo.pack(side=tk.LEFT, padx=6)
        ttk.Button(top, text="Yenile", command=self._refresh_ports).pack(side=tk.LEFT)
        ttk.Label(top, text="Baud:").pack(side=tk.LEFT, padx=(10, 0))
        self.baud_var = tk.StringVar(value=str(core.DEFAULT_BAUD))
        self.baud_combo = ttk.Combobox(top, textvariable=self.baud_var, width=8, state="readonly",
                                       values=[str(b) for b in core.BAUDS])
        self.baud_combo.pack(side=tk.LEFT, padx=4)
        ttk.Label(top, text="8N1").pack(side=tk.LEFT)
        self.conn_btn = ttk.Button(top, text="Bağlan", command=self._toggle_connect)
        self.conn_btn.pack(side=tk.LEFT, padx=6)
        self.baud_btn = ttk.Button(top, text="Hızı MCU'ya uygula", command=self._apply_baud)
        self.baud_btn.pack(side=tk.LEFT)
        self.baud_info = tk.StringVar(value="Kart resetten sonra 115200 ile başlar")
        ttk.Label(top, textvariable=self.baud_info, style="Hint.TLabel").pack(side=tk.LEFT, padx=8)
        self.status_var = tk.StringVar(value="Bağlı değil")
        ttk.Label(top, textvariable=self.status_var, style="Hint.TLabel").pack(side=tk.LEFT, padx=12)

        body = ttk.Frame(self.root, padding=(10, 0, 10, 10))
        body.pack(fill=tk.BOTH, expand=True)
        left = ttk.Frame(body, width=360)
        left.pack(side=tk.LEFT, fill=tk.Y)
        left.pack_propagate(False)
        right = ttk.Frame(body)
        right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(10, 0))

        # --- senaryo
        sf = ttk.LabelFrame(left, text="Senaryo", padding=8)
        sf.pack(fill=tk.X)
        self.scn_var = tk.IntVar(value=0)
        self.scn_radios = []
        for s, cfg in core.SCENARIOS.items():
            rb = ttk.Radiobutton(sf, text=cfg["label"], value=s, variable=self.scn_var)
            rb.pack(anchor=tk.W)
            ttk.Label(sf, text="      Hedef: " + cfg["goal"], style="Hint.TLabel").pack(anchor=tk.W)
            self.scn_radios.append(rb)
        self.apply_btn = ttk.Button(sf, text="Senaryoyu MCU'ya uygula", command=self._apply_scenario)
        self.apply_btn.pack(fill=tk.X, pady=(8, 2))
        self.mcu_scn_var = tk.StringVar(value="MCU senaryosu: bilinmiyor")
        ttk.Label(sf, textvariable=self.mcu_scn_var).pack(anchor=tk.W)

        # --- deney
        df = ttk.LabelFrame(left, text="Deney", padding=8)
        df.pack(fill=tk.X, pady=(8, 0))
        row = ttk.Frame(df)
        row.pack(fill=tk.X)
        self.rec_btn = ttk.Button(row, text="Kaydı başlat", command=self._toggle_record)
        self.rec_btn.pack(side=tk.LEFT, fill=tk.X, expand=True)
        self.cancel_btn = ttk.Button(row, text="İptal", command=self._cancel_record)
        self.cancel_btn.pack(side=tk.LEFT, padx=(6, 0))
        self.rec_var = tk.StringVar(value="Kayıt yok")
        ttk.Label(df, textvariable=self.rec_var).pack(anchor=tk.W, pady=(6, 0))
        frow = ttk.Frame(df)
        frow.pack(fill=tk.X, pady=(4, 0))
        try:
            shown = os.path.relpath(self.out_dir, PROJECT_DIR) + os.sep
        except ValueError:          # farkli surucu
            shown = self.out_dir
        ttk.Label(frow, text=shown, style="Hint.TLabel").pack(side=tk.LEFT)
        ttk.Button(frow, text="Klasörü aç", command=self._open_folder).pack(side=tk.RIGHT)

        # --- son buton olayi
        bf = ttk.LabelFrame(left, text="Son buton olayı (BTN)", padding=8)
        bf.pack(fill=tk.X, pady=(8, 0))
        self.pressed_lbl = tk.Label(bf, text="Bekleniyor...", font=("Segoe UI", 16, "bold"),
                                    bg="#EEEEEE", fg="#555555", pady=6)
        self.pressed_lbl.pack(fill=tk.X)
        self.evt_var = tk.StringVar(value="Olay: -   Senaryo: -")
        ttk.Label(bf, textvariable=self.evt_var, style="Mid.TLabel").pack(anchor=tk.W, pady=(6, 0))
        self.r_var = tk.StringVar(value="R = -")
        self.r_lbl = ttk.Label(bf, textvariable=self.r_var, style="Mid.TLabel")
        self.r_lbl.pack(anchor=tk.W)
        self.stage_var = tk.StringVar(value="")
        ttk.Label(bf, textvariable=self.stage_var, justify=tk.LEFT, font=("Consolas", 9)).pack(anchor=tk.W)

        # --- canli istatistik
        lf = ttk.LabelFrame(left, text="Kayıt istatistiği", padding=8)
        lf.pack(fill=tk.BOTH, expand=True, pady=(8, 0))
        self.stat_vars = {}
        rows = [
            ("ok", "Başarılı ölçüm"), ("R", "R min / ort / maks"), ("over", "> 20 ms (tamamlanan)"),
            ("drop", "Drop (buttonQ / txQ)"), ("txerr", "TX hata"), ("tmo", "Timeout"),
            ("lost", "Kayıt kaybı (MCU / PC)"), ("perr", "Bozuk satır"),
        ]
        for i, (key, text) in enumerate(rows):
            ttk.Label(lf, text=text + ":").grid(row=i, column=0, sticky=tk.W)
            v = tk.StringVar(value="-")
            ttk.Label(lf, textvariable=v, font=("Consolas", 10)).grid(row=i, column=1, sticky=tk.W, padx=(8, 0))
            self.stat_vars[key] = v

        # --- sag: sekmeler
        nb = ttk.Notebook(right)
        nb.pack(fill=tk.BOTH, expand=True)

        gtab = ttk.Frame(nb)
        nb.add(gtab, text="Grafikler")
        gbar = ttk.Frame(gtab, padding=(0, 6))
        gbar.pack(fill=tk.X)
        ttk.Label(gbar, text="Grafik 1 kaynağı:").pack(side=tk.LEFT)
        self.src_var = tk.StringVar(value="Canlı kayıt")
        self.src_combo = ttk.Combobox(gbar, textvariable=self.src_var, state="readonly", width=26)
        self.src_combo.pack(side=tk.LEFT, padx=6)
        self.src_combo.bind("<<ComboboxSelected>>", lambda e: self._mark_dirty())
        self.fig = Figure(figsize=(9, 7), dpi=100, constrained_layout=True)
        self.ax1 = self.fig.add_subplot(2, 1, 1)
        self.ax2 = self.fig.add_subplot(2, 1, 2)
        self.canvas = FigureCanvasTkAgg(self.fig, master=gtab)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        btab = ttk.Frame(nb)
        nb.add(btab, text="BTN mesajları")
        cols = ("id", "scn", "status", "R", "a", "b", "c", "d", "dl")
        heads = ("Olay", "Senaryo", "Durum", "R (ms)", "t1-t0 µs", "t2-t1 µs", "t3-t2 µs", "t4-t3 µs", "20 ms")
        self.btn_tree = self._tree(btab, cols, heads, (60, 70, 80, 80, 80, 80, 80, 80, 60))
        self.btn_tree.tag_configure("miss", foreground="#C62828")
        self.btn_tree.tag_configure("loss", foreground="#6A1B9A")

        ttab = ttk.Frame(nb)
        nb.add(ttab, text="TEL mesajları")
        self.tel_var = tk.StringVar(value="TEL: -")
        ttk.Label(ttab, textvariable=self.tel_var, padding=(4, 6)).pack(anchor=tk.W)
        self.tel_tree = self._tree(ttab, ("scn", "seq", "temp", "vbat", "exec"),
                                   ("Senaryo", "Sıra", "Sıcaklık (°C)", "Vbat (mV)", "Görev süresi (µs)"),
                                   (80, 80, 100, 100, 140))

        stab = ttk.Frame(nb)
        nb.add(stab, text="Özet")
        ttk.Label(stab, text="measurement/summary.csv  (diskteki s<n>.csv'lerden, her senaryo + baud için bir satır)",
                  style="Hint.TLabel", padding=(4, 6)).pack(anchor=tk.W)
        heads = {
            "scenario": "Senaryo", "baud": "Baud", "telemetry_period_ms": "TEL ms", "extra_work_ms": "Ek iş ms",
            "events": "Olay", "ok": "Başarılı", "R_min_ms": "R min", "R_avg_ms": "R ort", "R_max_ms": "R maks",
            "over_20ms": ">20ms", "drop_isr": "Drop ISR", "drop_txq": "Drop txQ", "tx_error": "TX hata",
            "timeout": "Timeout", "rec_lost_mcu": "Kayıp MCU", "rec_lost_pc": "Kayıp PC",
            "avg_isr_to_task_us": "t1-t0", "avg_task_us": "t2-t1", "avg_queue_us": "t3-t2", "avg_uart_us": "t4-t3",
        }
        self.sum_tree = self._tree(stab, tuple(core.SUMMARY_COLUMNS),
                                   tuple(heads[c] for c in core.SUMMARY_COLUMNS),
                                   tuple(70 for _ in core.SUMMARY_COLUMNS))

        rtab = ttk.Frame(nb)
        nb.add(rtab, text="Ham satırlar")
        self.raw = tk.Text(rtab, height=10, font=("Consolas", 9), state=tk.DISABLED)
        self.raw.pack(fill=tk.BOTH, expand=True)

    @staticmethod
    def _tree(parent, cols, heads, widths):
        frame = ttk.Frame(parent)
        frame.pack(fill=tk.BOTH, expand=True)
        tree = ttk.Treeview(frame, columns=cols, show="headings")
        for c, h, w in zip(cols, heads, widths):
            tree.heading(c, text=h)
            tree.column(c, width=w, anchor=tk.CENTER)
        sb = ttk.Scrollbar(frame, orient=tk.VERTICAL, command=tree.yview)
        tree.configure(yscrollcommand=sb.set)
        tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        sb.pack(side=tk.LEFT, fill=tk.Y)
        return tree

    # ------------------------------------------------------------------ port
    def _refresh_ports(self) -> None:
        ports = SerialLink.ports()
        values = [f"{dev} - {desc}" for dev, desc in ports]
        self.port_combo["values"] = values
        if values and not self.port_var.get():
            stlink = [v for v in values if "STLink" in v or "STMicro" in v]
            self.port_var.set((stlink or values)[0])

    def _toggle_connect(self) -> None:
        if self.link.is_open:
            self.link.close()
            self.status_var.set("Bağlantı kesildi")
        else:
            sel = self.port_var.get()
            if not sel:
                messagebox.showwarning("Port", "Önce bir port seçin.")
                return
            port = sel.split(" - ")[0]
            baud = int(self.baud_var.get())
            try:
                self.link.open(port, baud)
            except Exception as exc:
                messagebox.showerror("Port açılamadı", str(exc))
                return
            self.link_baud = baud
            self.status_var.set(f"Bağlı: {port} @ {baud}")
            self.scn_seen = False
            self.link.write("?\n")          # kayit yok -> durum sorgusu serbest
            self.root.after(1500, self._check_connect)
        self._update_buttons()

    def _check_connect(self) -> None:
        if self.link.is_open and not self.scn_seen and self.baud_sw is None:
            self.baud_info.set(f"MCU {self.link_baud} baud'da yanıt vermedi. Hız farklı olabilir "
                               "(kart resetten sonra 115200).")

    # ------------------------------------------------------------------ UART hizi
    def _apply_baud(self) -> None:
        if self.exp is not None or self.baud_sw is not None or not self.link.is_open:
            return
        new = int(self.baud_var.get())
        if new == self.link_baud:
            self.baud_info.set(f"Zaten {new} baud")
            return
        self.baud_sw = {"new": new, "old": self.link_baud, "phase": "ack", "t0": self.root.tk.call("clock", "milliseconds")}
        self.baud_info.set(f"{self.link_baud} -> {new} baud isteniyor...")
        self.link.write(f"B{new}\n")
        self._update_buttons()
        self.root.after(BAUD_ACK_MS, self._baud_ack_timeout)

    def _baud_ack_timeout(self) -> None:
        sw = self.baud_sw
        if sw is not None and sw["phase"] == "ack":
            self.baud_sw = None
            self.baud_info.set(f"MCU hız değişimini onaylamadı; {self.link_baud} baud'da kalındı.")
            self.baud_var.set(str(self.link_baud))
            self._update_buttons()

    def _on_baud_ack(self, f: dict) -> None:
        sw = self.baud_sw
        if sw is None or sw["phase"] != "ack" or f["new"] != sw["new"]:
            return
        sw["phase"] = "confirm"
        sw["t_ack"] = self.root.tk.call("clock", "milliseconds")
        self.link.set_baud(sw["new"])
        self.link_baud = sw["new"]
        self.root.after(150, lambda: self.link.write("?\n"))    # yeni hizda onay
        self.root.after(BAUD_CONFIRM_MS, self._baud_confirm_timeout)

    def _baud_confirm_timeout(self) -> None:
        sw = self.baud_sw
        if sw is None or sw["phase"] != "confirm":
            return
        # Yeni hizda konusulamadi: MCU kendiliginden eski hiza donecek
        sw["phase"] = "revert"
        self.link.set_baud(sw["old"])
        self.link_baud = sw["old"]
        self.baud_var.set(str(sw["old"]))
        self.baud_info.set(f"{sw['new']} baud çalışmadı (ST-LINK/USB desteklemiyor olabilir); "
                           f"{sw['old']} baud'a dönülüyor...")
        wait = max(100, MCU_REVERT_MS - BAUD_CONFIRM_MS)
        self.root.after(wait, self._baud_revert_done)

    def _baud_revert_done(self) -> None:
        sw = self.baud_sw
        self.baud_sw = None
        self.link.write("?\n")
        if sw is not None:
            self.baud_info.set(f"{sw['new']} baud çalışmadı; {sw['old']} baud'a dönüldü.")
        self._update_buttons()

    # ------------------------------------------------------------------ senaryo / deney
    def _apply_scenario(self) -> None:
        if self.exp is not None:
            return
        s = self.scn_var.get()
        self.link.write(f"S{s}\n")
        self.mcu_scn_var.set(f"MCU senaryosu: S{s} isteniyor...")

    def _toggle_record(self) -> None:
        if self.exp is None:
            self._start_record()
        else:
            self._stop_record()

    def _start_record(self) -> None:
        if self.baud_sw is not None:
            messagebox.showwarning("Baud", "Hız değişimi sürüyor, bitmesini bekleyin.")
            return
        if self.mcu_baud is not None and self.mcu_baud != self.link_baud:
            messagebox.showwarning("Baud", f"MCU {self.mcu_baud}, PC {self.link_baud} baud. Önce hızı eşitleyin.")
            return
        if self.mcu_scn is None:
            messagebox.showwarning("Senaryo", "MCU senaryosu bilinmiyor. Senaryoyu MCU'ya uygulayın.")
            return
        if self.scn_var.get() != self.mcu_scn:
            messagebox.showwarning("Senaryo",
                                   f"Seçili senaryo S{self.scn_var.get()}, MCU'da aktif olan S{self.mcu_scn}.\n"
                                   "Önce 'Senaryoyu MCU'ya uygula' ile eşitleyin.")
            return
        self.exp = core.Experiment(self.mcu_scn, self.next_expected_id, self.link_baud)
        self.src_var.set("Canlı kayıt")
        self._update_buttons()
        self._update_stats()
        self._mark_dirty()

    def _stop_record(self) -> None:
        exp = self.exp
        recs = exp.finalized()
        if not recs:
            if self.confirm("Kayıt boş", "Hiç olay kaydedilmedi. Kayıt kapatılsın mı?"):
                self.exp = None
                self._update_buttons()
            return
        key = (exp.scenario, exp.baud)
        path = core.scenario_csv_path(self.out_dir, exp.scenario)
        if self.disk.get(key):
            if not self.confirm("Üzerine yaz",
                                f"{os.path.basename(path)} içinde S{exp.scenario} @ {exp.baud} baud için zaten "
                                f"{len(self.disk[key])} kayıt var.\nYeni deneyle değiştirilsin mi? "
                                "(diğer hızlardaki kayıtlar korunur)"):
                return
        self.disk[key] = recs
        core.write_scenario_csv(self.out_dir, self.disk, exp.scenario)
        core.write_summary_csv(self.out_dir, self.disk)
        self.last_exp = exp
        self.exp = None
        self.rec_var.set(f"Kaydedildi: {os.path.basename(path)} @ {exp.baud} baud ({len(recs)} olay) + summary.csv")
        self.src_var.set(self._src_label(key))
        self._refresh_summary_table()
        self._update_buttons()
        self._mark_dirty()

    def _cancel_record(self) -> None:
        if self.exp is not None and self.confirm("İptal", "Kayıt kaydedilmeden atılsın mı?"):
            self.exp = None
            self.rec_var.set("Kayıt iptal edildi")
            self._update_buttons()
            self._mark_dirty()

    def _update_buttons(self) -> None:
        recording = self.exp is not None
        connected = self.link.is_open
        self.conn_btn.configure(text="Bağlantıyı kes" if connected else "Bağlan")
        # Kayit surerken MCU'ya hicbir sey gonderilmez
        idle = connected and not recording and self.baud_sw is None
        self.apply_btn.configure(state=tk.NORMAL if idle else tk.DISABLED)
        self.baud_btn.configure(state=tk.NORMAL if idle else tk.DISABLED)
        self.baud_combo.configure(state="disabled" if (recording or self.baud_sw is not None) else "readonly")
        for rb in self.scn_radios:
            rb.configure(state=tk.DISABLED if recording else tk.NORMAL)
        self.rec_btn.configure(text="Kaydı bitir ve CSV kaydet" if recording else "Kaydı başlat",
                               state=tk.NORMAL if (connected or recording) else tk.DISABLED)
        self.cancel_btn.configure(state=tk.NORMAL if recording else tk.DISABLED)
        if recording:
            self.rec_var.set(f"KAYIT: S{self.exp.scenario} @ {self.exp.baud} baud - {len(self.exp.records)} olay")

    def _open_folder(self) -> None:
        try:
            os.startfile(self.out_dir)  # type: ignore[attr-defined]
        except Exception as exc:
            messagebox.showinfo("Klasör", f"{self.out_dir}\n{exc}")

    # ------------------------------------------------------------------ gelen satirlar
    def _poll(self) -> None:
        try:
            for _ in range(500):
                kind, payload = self.q.get_nowait()
                if kind == "line":
                    self.handle_line(payload)
                else:
                    self.link.close()
                    self.status_var.set(f"Bağlantı hatası: {payload}")
                    self._update_buttons()
        except queue.Empty:
            pass
        self.root.after(50, self._poll)

    def handle_line(self, text: str) -> None:
        self._raw_append(text)
        try:
            msg = core.parse_line(text)
        except core.ParseError:
            if self.baud_sw is not None:      # hiz gecisindeki bozuk baytlar sayilmaz
                return
            self.parse_errors += 1
            if self.exp is not None:
                self.exp.parse_errors += 1
                self._update_stats()
            return
        if msg is None:
            return

        if msg.kind == "COMMENT":
            if self.exp is not None and "olcum" in msg.fields["text"]:
                self.status_var.set("UYARI: MCU yeniden başladı, kimlikler sıfırlandı.")
        elif msg.kind == "BAUD":
            self._on_baud_ack(msg.fields)
        elif msg.kind == "SCN":
            f = msg.fields
            self.scn_seen = True
            self.mcu_baud = f["baud"] if f["baud"] is not None else self.link_baud
            sw = self.baud_sw
            if sw is not None and sw["phase"] == "confirm" and self.mcu_baud == sw["new"]:
                self.baud_sw = None
                self.baud_info.set(f"UART hızı: {sw['new']} baud (MCU onayladı)")
                self._update_buttons()
            elif sw is None:
                self.baud_info.set(f"UART hızı: {self.mcu_baud} baud")
            self.mcu_scn = f["scenario"]
            self.next_expected_id = f["next_id"]
            tel = "kapalı" if f["period_ms"] == 0 else f"{f['period_ms']} ms"
            self.mcu_scn_var.set(f"MCU senaryosu: S{f['scenario']}  (TEL {tel}, ek iş {f['work_ms']} ms)")
            if self.exp is None:
                self.scn_var.set(f["scenario"])
        elif msg.kind == "TEL":
            self._on_tel(msg.fields)
        elif msg.kind == "BTN":
            self._on_btn(msg.fields)
        elif msg.kind == "REC":
            self._on_rec(msg.record)

    def _on_tel(self, f: dict) -> None:
        self.tel_count[f["scenario"]] += 1
        self.tel_exec_max = max(self.tel_exec_max, f["exec_us"])
        self.tel_exec_sum += f["exec_us"]
        self.tel_exec_n += 1
        self._tree_insert(self.tel_tree, (f"S{f['scenario']}", f["seq"], f"{f['temp_c']:.1f}",
                                          f["vbat_mv"], f["exec_us"]))
        if f["seq"] % 10 == 0:
            self.tel_var.set(
                f"TEL (senaryo S{f['scenario']}): toplam {self.tel_count[f['scenario']]} mesaj, "
                f"görev süresi son {f['exec_us']} µs / ort {self.tel_exec_sum / self.tel_exec_n:.0f} µs / "
                f"maks {self.tel_exec_max} µs   (ek iş olmasa da görev süresi sıfır değildir)")

    def _on_btn(self, f: dict) -> None:
        self.pressed_lbl.configure(text="Butona basıldı", bg="#2E7D32", fg="white")
        self.evt_var.set(f"Olay: #{f['id']}   Senaryo: S{f['scenario']}")
        self.r_var.set("R = (REC bekleniyor)")
        if self.flash_job is not None:
            self.root.after_cancel(self.flash_job)
        self.flash_job = self.root.after(
            700, lambda: self.pressed_lbl.configure(bg="#C8E6C9", fg="#1B5E20"))

    def _on_rec(self, rec: core.Record) -> None:
        self.next_expected_id = rec.id + 1
        self.evt_var.set(f"Olay: #{rec.id}   Senaryo: S{rec.scenario}")
        if rec.ok:
            miss = rec.deadline_miss
            self.r_var.set(f"R = {rec.R_us / 1000:.3f} ms   {'DEADLINE AŞILDI' if miss else 'OK'} (20 ms)")
            self.stage_var.set("\n".join(f"{name:<28}{getattr(rec, key):>8} µs" for key, name in core.STAGES))
        else:
            self.r_var.set(f"R = yok   durum: {rec.status}")
            self.stage_var.set("")
            self.pressed_lbl.configure(text=f"Olay #{rec.id}: {rec.status}", bg="#6A1B9A", fg="white")

        def us(x):
            return "" if x is None else x
        tag = ("miss",) if rec.deadline_miss else (() if rec.ok else ("loss",))
        self._tree_insert(self.btn_tree, (
            rec.id, f"S{rec.scenario}", rec.status,
            "" if rec.R_us is None else f"{rec.R_us / 1000:.3f}",
            us(rec.isr_to_task_us), us(rec.task_us), us(rec.queue_us), us(rec.uart_us),
            "" if rec.deadline_miss is None else ("AŞILDI" if rec.deadline_miss else "OK"),
        ), tag)

        if self.exp is not None:
            self.exp.add(rec)
            self._update_buttons()
            self._update_stats()
            self._mark_dirty()

    # ------------------------------------------------------------------ yardimcilar
    def _tree_insert(self, tree, values, tags=()) -> None:
        tree.insert("", 0, values=values, tags=tags)
        children = tree.get_children()
        if len(children) > MAX_TREE_ROWS:
            tree.delete(*children[MAX_TREE_ROWS:])

    def _raw_append(self, text: str) -> None:
        self.raw.configure(state=tk.NORMAL)
        self.raw.insert(tk.END, text + "\n")
        lines = int(self.raw.index("end-1c").split(".")[0])
        if lines > MAX_RAW_LINES:
            self.raw.delete("1.0", f"{lines - MAX_RAW_LINES}.0")
        self.raw.see(tk.END)
        self.raw.configure(state=tk.DISABLED)

    def _update_stats(self) -> None:
        if self.exp is None:
            return
        s = core.summarize(self.exp.scenario, self.exp.finalized(), self.exp.baud)

        def f(x):
            return "-" if x is None else f"{x:.3f}"
        self.stat_vars["ok"].set(f"{s['ok']} / {s['events']} olay")
        self.stat_vars["R"].set(f"{f(s['R_min_ms'])} / {f(s['R_avg_ms'])} / {f(s['R_max_ms'])} ms")
        self.stat_vars["over"].set(str(s["over_20ms"]))
        self.stat_vars["drop"].set(f"{s['drop_isr']} / {s['drop_txq']}")
        self.stat_vars["txerr"].set(str(s["tx_error"]))
        self.stat_vars["tmo"].set(str(s["timeout"]))
        self.stat_vars["lost"].set(f"{s['rec_lost_mcu']} / {s['rec_lost_pc']}")
        self.stat_vars["perr"].set(f"{self.exp.parse_errors}  (başka senaryo: {self.exp.foreign})")

    def _refresh_summary_table(self) -> None:
        self.sum_tree.delete(*self.sum_tree.get_children())
        for row in core.summary_rows(self.disk):
            self.sum_tree.insert("", tk.END, values=tuple("" if row[c] is None else row[c]
                                                          for c in core.SUMMARY_COLUMNS))

    def _mark_dirty(self) -> None:
        self.dirty = True

    @staticmethod
    def _src_label(key) -> str:
        return f"s{key[0]}.csv @ {key[1]} baud"

    def _refresh_src_values(self) -> None:
        keys = sorted(k for k, v in self.disk.items() if v)
        self.src_map = {self._src_label(k): k for k in keys}
        self.src_combo["values"] = ["Canlı kayıt"] + list(self.src_map)

    def _redraw_tick(self) -> None:
        if self.dirty:
            self.dirty = False
            self.redraw()
        self.root.after(REDRAW_MS, self._redraw_tick)

    # ------------------------------------------------------------------ grafikler
    def _chart1_source(self):
        src = self.src_var.get()
        if src == "Canlı kayıt":
            exp = self.exp or self.last_exp
            if exp is None:
                return None, []
            return (exp.scenario, exp.baud), exp.finalized()
        key = getattr(self, "src_map", {}).get(src)
        return (key, self.disk.get(key, [])) if key else (None, [])

    def redraw(self) -> None:
        self._refresh_src_values()
        self._draw_response_chart()
        self._draw_stage_chart()
        self.canvas.draw_idle()

    def _draw_response_chart(self) -> None:
        ax = self.ax1
        ax.clear()
        scn, recs = self._chart1_source()
        title = "Grafik 1 - Olay başına yanıt süresi R = t4 − t0"
        if scn is None or not recs:
            ax.set_title(title + "  (veri yok)")
            ax.axhline(core.DEADLINE_US / 1000, color="#C62828", ls="--", lw=1.2)
            ax.set_xlabel("Olay kimliği")
            ax.set_ylabel("R (ms)")
            return
        ok = [r for r in recs if r.ok]
        if ok:
            x = [r.id for r in ok]
            y = [r.R_us / 1000 for r in ok]
            # Cizgi yalnizca ardisik basarili olaylari baglar; kayipta kesilir
            by_id = {r.id: r.R_us / 1000 for r in ok}
            ids = range(recs[0].id, recs[-1].id + 1)
            ax.plot(list(ids), [by_id.get(i, float("nan")) for i in ids], color="#4C78A8", lw=1, alpha=0.6)
            inside = [(a, b) for a, b in zip(x, y) if b <= core.DEADLINE_US / 1000]
            over = [(a, b) for a, b in zip(x, y) if b > core.DEADLINE_US / 1000]
            if inside:
                ax.scatter(*zip(*inside), s=18, color="#4C78A8", zorder=3, label=f"Tamamlanan ({len(inside)})")
            if over:
                ax.scatter(*zip(*over), s=30, color="#C62828", zorder=4, label=f"> 20 ms ({len(over)})")
        ax.axhline(core.DEADLINE_US / 1000, color="#C62828", ls="--", lw=1.2, label="Deadline 20 ms")
        for status, (marker, color, label) in LOSS_STYLE.items():
            ids = [r.id for r in recs if r.status == status]
            if ids:
                ax.scatter(ids, [0] * len(ids), marker=marker, s=60, color=color, zorder=5,
                           clip_on=False, label=f"{label} ({len(ids)})")
        ymax = max([core.DEADLINE_US / 1000] + [r.R_us / 1000 for r in ok]) * 1.15
        ax.set_ylim(-ymax * 0.04, ymax)
        ax.set_title(f"{title}  -  S{scn[0]} @ {scn[1]} baud ({len(ok)}/{len(recs)} başarılı)")
        ax.set_xlabel("Olay kimliği")
        ax.set_ylabel("R (ms)")
        ax.grid(alpha=0.3)
        ax.legend(loc="upper left", fontsize=8, ncol=2)

    def _draw_stage_chart(self) -> None:
        ax = self.ax2
        ax.clear()
        data = {k: v for k, v in self.disk.items() if v}
        live = (self.exp.scenario, self.exp.baud) if (self.exp is not None and self.exp.records) else None
        if live is not None:
            data[live] = self.exp.finalized()
        # Veri olan her (senaryo, baud) bir sutun; hic veri yoksa S0..S5 bos gosterilir
        keys = sorted(data) or [(s, core.DEFAULT_BAUD) for s in range(core.SCENARIO_COUNT)]

        labels, bottoms = [], []
        stage_vals = {k: [] for k, _ in core.STAGES}
        notes = []
        for key in keys:
            summ = core.summarize(key[0], data.get(key, []), key[1])
            labels.append(f"S{key[0]}\n{key[1]}" + ("\n(canlı)" if key == live else ""))
            for key, _ in core.STAGES:
                v = summ["avg_" + key]
                stage_vals[key].append(0.0 if v is None else v / 1000)
            notes.append(summ)
            bottoms.append(0.0)

        x = range(len(keys))
        for (key, name), color in zip(core.STAGES, STAGE_COLORS):
            vals = stage_vals[key]
            ax.bar(x, vals, bottom=bottoms, color=color, width=0.6, label=name)
            bottoms = [b + v for b, v in zip(bottoms, vals)]
        for i, summ in enumerate(notes):
            if summ["ok"]:
                ax.text(i, bottoms[i], f"R ort {summ['R_avg_ms']:.2f} ms\nn={summ['ok']}",
                        ha="center", va="bottom", fontsize=8)
            else:
                ax.text(i, 0, "veri yok", ha="center", va="bottom", fontsize=8, color="#888888")
        ax.set_xticks(list(x), labels)
        top = max(bottoms + [1.0])
        ax.set_ylim(0, top * 1.3)
        ax.set_ylabel("Ortalama süre (ms)")
        ax.set_title("Grafik 2 - Senaryo ve baud başına aşamaların ortalama süreleri (başarılı ölçümler)")
        ax.grid(axis="y", alpha=0.3)
        ax.legend(loc="upper left", fontsize=8, ncol=2)

    # ------------------------------------------------------------------ kapanis
    def _on_close(self) -> None:
        if self.exp is not None and not self.confirm("Çıkış", "Kayıt sürüyor ve kaydedilmedi. Çıkılsın mı?"):
            return
        self.link.close()
        self.root.destroy()


# ----------------------------------------------------------------------------- selftest
def selftest() -> int:
    """Donanimsiz: sentetik satirlarla tum yollari calistirir, CSV ve grafik uretir."""
    out = tempfile.mkdtemp(prefix="hafta01_gui_")
    root = tk.Tk()
    app = App(root, out)
    app.confirm = lambda *a, **k: True

    def t(base, *deltas):
        vals, cur = [base], base
        for d in deltas:
            cur = (cur + d) & core.U32
            vals.append(cur)
        return vals

    app.handle_line("# Hafta-01 olcum: TIM2=1MHz DL=20000us filtre=30000us")
    app.handle_line("SCN,3,10,0,0,115200")
    app.scn_var.set(3)
    app.link.ser = object()          # "bagli" gibi davran (yazma yok)
    app._start_record()
    for i in range(6):
        app.handle_line(f"TEL,3,{i},{240 + i},3300,{12 + i}")
    app.handle_line("BTN,3,0")
    app.handle_line("REC,3,0,OK," + ",".join(map(str, t(1000, 15, 4, 2100, 1900))))
    app.handle_line("REC,3,1,OK," + ",".join(map(str, t(core.U32 - 500, 20, 5, 9000, 2000))))  # sarma
    app.handle_line("REC,3,2,DROPQ," + ",".join(map(str, t(50000, 10, 3))) + ",0,0")
    app.handle_line("REC,3,4,OK," + ",".join(map(str, t(90000, 12, 4, 21000, 2100))))          # >20 ms, id 3 kayip
    app.handle_line("REC,3,5,TMO," + ",".join(map(str, t(150000, 12, 4, 100))) + ",0")
    app.handle_line("REC,9,6,OK,1,2,3,4,5")                                                       # bozuk
    app.redraw()
    app._stop_record()
    # Ayni senaryo 9600 baud'da: hiz degisim akisi (port yazmasi yok sayilir)
    app.link.write = lambda text: None
    app.link.set_baud = lambda b: None
    app.baud_var.set("9600")
    app._apply_baud()
    app.handle_line("BAUD,9600,115200")
    assert app.baud_sw["phase"] == "confirm" and app.link_baud == 9600
    app.handle_line("SCN,3,10,0,6,9600")
    assert app.baud_sw is None and app.mcu_baud == 9600
    app._start_record()
    app.handle_line("REC,3,6,OK," + ",".join(map(str, t(400000, 30, 4, 25000, 9400))))   # 9600: UART yavas
    app.handle_line("REC,3,7,DROPQ," + ",".join(map(str, t(500000, 25, 4))) + ",0,0")
    app._stop_record()
    app.src_var.set("s3.csv @ 9600 baud")
    app.redraw()
    app.fig.savefig(os.path.join(out, "selftest.png"))
    root.update()
    summary = open(core.summary_csv_path(out), encoding="utf-8").read()
    s3 = open(core.scenario_csv_path(out, 3), encoding="utf-8").read()
    app.link.ser = None
    root.destroy()
    print("cikti klasoru:", out)
    print(s3)
    print(summary)
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default=DEFAULT_OUT, help="CSV klasoru (varsayilan: proje/measurement)")
    ap.add_argument("--selftest", action="store_true", help="donanimsiz duman testi")
    args = ap.parse_args()
    if args.selftest:
        return selftest()
    if serial is None:
        print("pyserial gerekli:  py -m pip install -r gui/requirements.txt", file=sys.stderr)
        return 1
    root = tk.Tk()
    App(root, os.path.abspath(args.out))
    root.mainloop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
