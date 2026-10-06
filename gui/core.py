"""
Hafta-01 olcum arayuzu - cekirdek mantik (GUI'den bagimsiz, test edilebilir).

Tum zamanlar MCU'da TIM2 (1 MHz, 32-bit) ile alinir; PC saati hicbir olcumde kullanilmaz.
Satir protokolu (MCU -> PC), main.c basindaki aciklamayla aynidir:

    SCN,<scn>,<period_ms>,<work_ms>,<next_id>[,<baud>]
    BAUD,<yeni>,<eski>          (hiz degisimi onayi, ESKI hizda gelir)
    TEL,<scn>,<seq>,<temp_dC>,<vbat_mV>,<exec_us>
    BTN,<scn>,<id>
    REC,<scn>,<id>,<status>,<t0>,<t1>,<t2>,<t3>,<t4>
    # yorum

Yanit suresi R = t4 - t0 (modulo 2^32):
    t0  buton ISR girisi (filtrenin kabul ettigi kenar)
    t1  ButtonTask olayi aldiktan hemen sonra
    t2  yanit icin xQueueSend'den hemen once
    t3  UART baslatma cagrisindan hemen once (ilk fiziksel bit degil)
    t4  UART TC tamamlanmasi islenirken (son bitten sonraki callback)
"""
from __future__ import annotations

import csv
import os
from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Optional, Tuple

U32 = 0xFFFFFFFF
DEADLINE_US = 20_000
SCENARIO_COUNT = 6

# Firmware'deki kBaud tablosuyla ayni. Eski (baud sutunsuz) kayitlar 115200'de alinmisti.
BAUDS = (9600, 19200, 57600, 115200, 230400, 460800, 921600)
DEFAULT_BAUD = 115200

# Firmware'deki kScenario tablosuyla ayni
SCENARIOS = {
    0: dict(period_ms=0,   work_ms=0, label="S0 - Telemetri kapalı",            goal="Referans yanıt süresi"),
    1: dict(period_ms=100, work_ms=0, label="S1 - 10 Hz (100 ms)",              goal="Düşük telemetri sıklığı"),
    2: dict(period_ms=20,  work_ms=0, label="S2 - 50 Hz (20 ms)",               goal="Orta telemetri sıklığı"),
    3: dict(period_ms=10,  work_ms=0, label="S3 - 100 Hz (10 ms)",              goal="Yüksek telemetri sıklığı"),
    4: dict(period_ms=10,  work_ms=2, label="S4 - 100 Hz + ~2 ms CPU işi",      goal="Ek CPU yükü"),
    5: dict(period_ms=10,  work_ms=5, label="S5 - 100 Hz + ~5 ms CPU işi",      goal="Daha fazla CPU yükü"),
}

# REC durumlari
OK = "OK"
DROPI = "DROPI"       # buttonQ dolu (ISR'de)
DROPQ = "DROPQ"       # yanit txQ'ya sigmadi
TXERR = "TXERR"       # UART baslatma hatasi
TMO = "TMO"           # TC zamaninda gelmedi
LOST = "LOST"         # MCU'da kayit slotu raporlanmadan ustune yazildi
LOST_PC = "LOST_PC"   # MCU raporladi ama PC'ye ulasmadi / bozuk geldi (kimlik boslugu)
STATUSES = (OK, DROPI, DROPQ, TXERR, TMO, LOST, LOST_PC)

STAGES = (
    ("isr_to_task_us", "ISR -> ButtonTask (t1-t0)"),
    ("task_us",        "ButtonTask işleme (t2-t1)"),
    ("queue_us",       "txQ bekleme (t3-t2)"),
    ("uart_us",        "UART gönderim (t4-t3)"),
)

RECORD_COLUMNS = [
    "scenario", "baud", "id", "status", "t0_us", "t1_us", "t2_us", "t3_us", "t4_us",
    "isr_to_task_us", "task_us", "queue_us", "uart_us", "R_us", "deadline_miss",
]

SUMMARY_COLUMNS = [
    "scenario", "baud", "telemetry_period_ms", "extra_work_ms",
    "events", "ok", "R_min_ms", "R_avg_ms", "R_max_ms", "over_20ms",
    "drop_isr", "drop_txq", "tx_error", "timeout", "rec_lost_mcu", "rec_lost_pc",
    "avg_isr_to_task_us", "avg_task_us", "avg_queue_us", "avg_uart_us",
]


def u32diff(start: int, end: int) -> int:
    """end - start, TIM2'nin 32-bit sarmasina dayanikli."""
    return (end - start) & U32


# --------------------------------------------------------------------------- satirlar
@dataclass
class Record:
    scenario: int
    id: int
    status: str
    t0: int = 0
    t1: int = 0
    t2: int = 0
    t3: int = 0
    t4: int = 0
    baud: int = DEFAULT_BAUD

    # Bir asamanin suresi ancak iki ucu da olculduyse vardir
    def _stage(self, a: str, b: str) -> Optional[int]:
        need = {
            "t1": (OK, DROPQ, TXERR, TMO),
            "t2": (OK, DROPQ, TXERR, TMO),
            "t3": (OK, TXERR, TMO),
            "t4": (OK,),
        }
        if self.status not in need[b]:
            return None
        return u32diff(getattr(self, a), getattr(self, b))

    @property
    def isr_to_task_us(self) -> Optional[int]:
        return self._stage("t0", "t1")

    @property
    def task_us(self) -> Optional[int]:
        return self._stage("t1", "t2")

    @property
    def queue_us(self) -> Optional[int]:
        return self._stage("t2", "t3")

    @property
    def uart_us(self) -> Optional[int]:
        return self._stage("t3", "t4")

    @property
    def ok(self) -> bool:
        return self.status == OK

    @property
    def R_us(self) -> Optional[int]:
        return u32diff(self.t0, self.t4) if self.ok else None

    @property
    def deadline_miss(self) -> Optional[bool]:
        r = self.R_us
        return None if r is None else r > DEADLINE_US

    def to_row(self) -> dict:
        def v(x):
            return "" if x is None else x
        times_known = self.status not in (LOST, LOST_PC)
        miss = self.deadline_miss
        return {
            "scenario": self.scenario, "baud": self.baud, "id": self.id, "status": self.status,
            "t0_us": self.t0 if times_known else "",
            "t1_us": self.t1 if self.isr_to_task_us is not None else "",
            "t2_us": self.t2 if self.task_us is not None else "",
            "t3_us": self.t3 if self.status in (OK, TXERR, TMO) else "",
            "t4_us": self.t4 if self.ok else "",
            "isr_to_task_us": v(self.isr_to_task_us), "task_us": v(self.task_us),
            "queue_us": v(self.queue_us), "uart_us": v(self.uart_us),
            "R_us": v(self.R_us), "deadline_miss": "" if miss is None else int(miss),
        }

    @staticmethod
    def from_row(row: dict) -> "Record":
        def i(key):
            s = (row.get(key) or "").strip()
            return int(s) if s else 0
        return Record(int(row["scenario"]), int(row["id"]), row["status"].strip(),
                      i("t0_us"), i("t1_us"), i("t2_us"), i("t3_us"), i("t4_us"),
                      i("baud") or DEFAULT_BAUD)


@dataclass
class Msg:
    kind: str                     # SCN | BAUD | TEL | BTN | REC | COMMENT
    fields: dict = field(default_factory=dict)
    record: Optional[Record] = None


class ParseError(ValueError):
    pass


def parse_line(line: str) -> Optional[Msg]:
    """Bir satiri cozer. Bos satir -> None. Bozuk satir -> ParseError."""
    line = line.strip()
    if not line:
        return None
    if line.startswith("#"):
        return Msg("COMMENT", {"text": line[1:].strip()})

    p = line.split(",")
    kind = p[0]
    try:
        if kind == "SCN" and len(p) in (5, 6):
            v = [int(x) for x in p[1:]]
            _check_scn(v[0])
            return Msg("SCN", dict(scenario=v[0], period_ms=v[1], work_ms=v[2], next_id=v[3],
                                   baud=v[4] if len(v) == 5 else None))
        if kind == "BAUD" and len(p) == 3:
            new, old = int(p[1]), int(p[2])
            return Msg("BAUD", dict(new=new, old=old))
        if kind == "TEL" and len(p) == 6:
            scn, seq, temp, vbat, exec_us = (int(x) for x in p[1:])
            _check_scn(scn)
            return Msg("TEL", dict(scenario=scn, seq=seq, temp_c=temp / 10.0, vbat_mv=vbat, exec_us=exec_us))
        if kind == "BTN" and len(p) == 3:
            scn, eid = int(p[1]), int(p[2])
            _check_scn(scn)
            return Msg("BTN", dict(scenario=scn, id=eid))
        if kind == "REC" and len(p) == 9:
            scn, eid, status = int(p[1]), int(p[2]), p[3]
            _check_scn(scn)
            if status not in STATUSES or status == LOST_PC:
                raise ParseError(f"bilinmeyen durum: {status}")
            t = [int(x) for x in p[4:9]]
            if any(x < 0 or x > U32 for x in t):
                raise ParseError("zaman damgasi 32-bit disinda")
            rec = Record(scn, eid, status, *t)
            return Msg("REC", dict(scenario=scn, id=eid, status=status), rec)
    except ValueError as exc:
        raise ParseError(f"{line!r}: {exc}") from exc
    raise ParseError(f"tanimsiz satir: {line!r}")


def _check_scn(scn: int) -> None:
    if not 0 <= scn < SCENARIO_COUNT:
        raise ValueError(f"senaryo araligi disinda: {scn}")


# --------------------------------------------------------------------------- deney
class Experiment:
    """Bir senaryonun belirli bir UART hizindaki tek kaydi. Kimlik bosluklari PC tarafi kayit kaybidir."""

    def __init__(self, scenario: int, start_id: Optional[int] = None, baud: int = DEFAULT_BAUD):
        self.scenario = scenario
        self.baud = baud
        self.start_id = start_id
        self.records: Dict[int, Record] = {}
        self.parse_errors = 0
        self.foreign = 0          # baska senaryoya ait REC (karismasin diye sayilip atilir)

    def add(self, rec: Record) -> bool:
        if rec.scenario != self.scenario:
            self.foreign += 1
            return False
        if self.start_id is not None and rec.id < self.start_id:
            return False          # kayit baslamadan onceki olay
        rec.baud = self.baud      # REC satiri hizi tasimaz; kaydin hizi gecerlidir
        self.records[rec.id] = rec
        return True

    def finalized(self) -> List[Record]:
        """Kimlik sirasina dizili kayitlar + aradaki eksik kimlikler LOST_PC olarak."""
        if not self.records:
            return []
        lo = self.start_id if self.start_id is not None else min(self.records)
        hi = max(self.records)
        out = []
        for i in range(lo, hi + 1):
            out.append(self.records.get(i) or Record(self.scenario, i, LOST_PC, baud=self.baud))
        return out


# --------------------------------------------------------------------------- ozet
def _avg(xs: List[int]) -> Optional[float]:
    return sum(xs) / len(xs) if xs else None


def summarize(scenario: int, records: Iterable[Record], baud: Optional[int] = None) -> dict:
    recs = list(records)
    ok = [r for r in recs if r.ok]
    R = [r.R_us for r in ok]
    cnt = {s: sum(1 for r in recs if r.status == s) for s in STATUSES}
    cfg = SCENARIOS[scenario]

    def ms(x):
        return None if x is None else round(x / 1000.0, 3)

    def us(x):
        return None if x is None else round(x, 1)

    out = {
        "scenario": f"S{scenario}",
        "baud": baud,
        "telemetry_period_ms": cfg["period_ms"] or "off",
        "extra_work_ms": cfg["work_ms"],
        "events": len(recs),
        "ok": len(ok),
        "R_min_ms": ms(min(R)) if R else None,
        "R_avg_ms": ms(_avg(R)),
        "R_max_ms": ms(max(R)) if R else None,
        "over_20ms": sum(1 for x in R if x > DEADLINE_US),
        "drop_isr": cnt[DROPI],
        "drop_txq": cnt[DROPQ],
        "tx_error": cnt[TXERR],
        "timeout": cnt[TMO],
        "rec_lost_mcu": cnt[LOST],
        "rec_lost_pc": cnt[LOST_PC],
    }
    for key, _ in STAGES:
        out["avg_" + key] = us(_avg([getattr(r, key) for r in ok]))
    return out


# --------------------------------------------------------------------------- CSV
def scenario_csv_path(folder: str, scenario: int) -> str:
    return os.path.join(folder, f"s{scenario}.csv")


def summary_csv_path(folder: str) -> str:
    return os.path.join(folder, "summary.csv")


def write_records_csv(path: str, records: Iterable[Record]) -> None:
    tmp = path + ".tmp"
    with open(tmp, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=RECORD_COLUMNS)
        w.writeheader()
        for r in records:
            w.writerow(r.to_row())
    os.replace(tmp, path)


def read_records_csv(path: str) -> List[Record]:
    if not os.path.exists(path):
        return []
    with open(path, newline="", encoding="utf-8") as f:
        return [Record.from_row(row) for row in csv.DictReader(f) if row.get("id")]


Key = Tuple[int, int]   # (senaryo, baud)


def load_all(folder: str) -> Dict[Key, List[Record]]:
    """Tum s<n>.csv'leri okur; kayitlari (senaryo, baud) ciftine gore ayirir."""
    out: Dict[Key, List[Record]] = {}
    for s in range(SCENARIO_COUNT):
        for r in read_records_csv(scenario_csv_path(folder, s)):
            out.setdefault((s, r.baud), []).append(r)
    return out


def write_scenario_csv(folder: str, data: Dict[Key, List[Record]], scenario: int) -> None:
    """s<n>.csv: bu senaryonun tum hizlardaki kayitlari (hiza, sonra kimlige gore)."""
    recs = [r for (s, b) in sorted(data) if s == scenario for r in data[(s, b)]]
    write_records_csv(scenario_csv_path(folder, scenario), recs)


def summary_rows(data: Dict[Key, List[Record]]) -> List[dict]:
    """Veri olan her (senaryo, baud) icin bir satir; hic verisi olmayan senaryo icin bos satir."""
    rows = []
    for s in range(SCENARIO_COUNT):
        bauds = sorted(b for (ss, b) in data if ss == s and data[(ss, b)])
        if not bauds:
            rows.append(summarize(s, [], None))
        for b in bauds:
            rows.append(summarize(s, data[(s, b)], b))
    return rows


def write_summary_csv(folder: str, data: Dict[Key, List[Record]]) -> List[dict]:
    rows = summary_rows(data)
    path = summary_csv_path(folder)
    tmp = path + ".tmp"
    with open(tmp, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=SUMMARY_COLUMNS)
        w.writeheader()
        for row in rows:
            w.writerow({k: ("" if v is None else v) for k, v in row.items()})
    os.replace(tmp, path)
    return rows


def ensure_measurement_folder(folder: str) -> None:
    """Klasoru ve bos (yalnizca baslikli) CSV'leri olusturur; mevcutlara dokunmaz."""
    os.makedirs(folder, exist_ok=True)
    for s in range(SCENARIO_COUNT):
        p = scenario_csv_path(folder, s)
        if not os.path.exists(p):
            write_records_csv(p, [])
    if not os.path.exists(summary_csv_path(folder)):
        write_summary_csv(folder, {})
