"""py -m unittest gui/test_core.py"""
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import core  # noqa: E402


class TestCore(unittest.TestCase):
    def test_u32diff_wraps(self):
        self.assertEqual(core.u32diff(0xFFFFFFF0, 0x10), 0x20)
        self.assertEqual(core.u32diff(5, 5), 0)

    def test_parse_rec_ok_and_stages(self):
        m = core.parse_line("REC,3,7,OK,100,115,119,2219,4119\r\n")
        r = m.record
        self.assertEqual((r.scenario, r.id, r.status), (3, 7, "OK"))
        self.assertEqual((r.isr_to_task_us, r.task_us, r.queue_us, r.uart_us), (15, 4, 2100, 1900))
        self.assertEqual(r.R_us, 4019)
        self.assertFalse(r.deadline_miss)

    def test_partial_stages_by_status(self):
        r = core.parse_line("REC,1,2,DROPQ,10,20,25,0,0").record
        self.assertEqual((r.isr_to_task_us, r.task_us), (10, 5))
        self.assertIsNone(r.queue_us)
        self.assertIsNone(r.R_us)
        r = core.parse_line("REC,1,3,TMO,10,20,25,40,0").record
        self.assertEqual(r.queue_us, 15)
        self.assertIsNone(r.uart_us)
        r = core.parse_line("REC,1,4,DROPI,10,0,0,0,0").record
        self.assertIsNone(r.isr_to_task_us)

    def test_parse_other_lines(self):
        self.assertEqual(core.parse_line("SCN,4,10,2,12").fields["work_ms"], 2)
        self.assertEqual(core.parse_line("TEL,3,9,251,3290,14").fields["temp_c"], 25.1)
        self.assertEqual(core.parse_line("BTN,0,5").fields["id"], 5)
        self.assertEqual(core.parse_line("# yorum").kind, "COMMENT")
        self.assertIsNone(core.parse_line("   "))

    def test_parse_errors(self):
        for bad in ("REC,3,7,OK,1,2,3", "REC,7,1,OK,1,2,3,4,5", "REC,1,1,WAT,1,2,3,4,5",
                    "REC,1,1,OK,1,2,3,4,99999999999", "TEL,x", "XYZ", "REC,1,1,LOST_PC,0,0,0,0,0"):
            with self.assertRaises(core.ParseError, msg=bad):
                core.parse_line(bad)

    def test_experiment_gaps_and_filters(self):
        e = core.Experiment(2, start_id=10)
        e.add(core.Record(2, 9, "OK", 0, 1, 2, 3, 4))      # kayit oncesi -> yok sayilir
        e.add(core.Record(2, 10, "OK", 0, 1, 2, 3, 4))
        e.add(core.Record(2, 13, "OK", 0, 1, 2, 3, 4))
        self.assertFalse(e.add(core.Record(1, 14, "OK", 0, 1, 2, 3, 4)))
        ids = [(r.id, r.status) for r in e.finalized()]
        self.assertEqual(ids, [(10, "OK"), (11, "LOST_PC"), (12, "LOST_PC"), (13, "OK")])
        self.assertEqual(e.foreign, 1)

    def test_summary(self):
        recs = [
            core.Record(4, 0, "OK", 0, 10, 12, 1012, 3012),       # R 3.012 ms
            core.Record(4, 1, "OK", 0, 10, 12, 20012, 22012),     # R 22.012 ms > 20
            core.Record(4, 2, "DROPI", 5),
            core.Record(4, 3, "DROPQ", 0, 1, 2),
            core.Record(4, 4, "TXERR", 0, 1, 2, 3),
            core.Record(4, 5, "TMO", 0, 1, 2, 3),
            core.Record(4, 6, "LOST"),
            core.Record(4, 7, "LOST_PC"),
        ]
        s = core.summarize(4, recs)
        self.assertEqual((s["events"], s["ok"], s["over_20ms"]), (8, 2, 1))
        self.assertEqual((s["R_min_ms"], s["R_max_ms"], s["R_avg_ms"]), (3.012, 22.012, 12.512))
        self.assertEqual((s["drop_isr"], s["drop_txq"], s["tx_error"], s["timeout"]), (1, 1, 1, 1))
        self.assertEqual((s["rec_lost_mcu"], s["rec_lost_pc"]), (1, 1))
        self.assertEqual(s["avg_queue_us"], 10500.0)
        self.assertEqual(s["extra_work_ms"], 2)
        empty = core.summarize(0, [])
        self.assertEqual((empty["ok"], empty["R_avg_ms"], empty["telemetry_period_ms"]), (0, None, "off"))

    def test_csv_roundtrip(self):
        d = tempfile.mkdtemp()
        core.ensure_measurement_folder(d)
        self.assertEqual(sorted(os.listdir(d)), ["s0.csv", "s1.csv", "s2.csv", "s3.csv", "s4.csv",
                                                 "s5.csv", "summary.csv"])
        recs = [core.Record(1, 0, "OK", 0xFFFFFF00, 0xFFFFFF10, 0xFFFFFF20, 0x30, 0x400),
                core.Record(1, 1, "DROPQ", 5, 6, 7), core.Record(1, 2, "LOST_PC")]
        core.write_records_csv(core.scenario_csv_path(d, 1), recs)
        back = core.load_all(d)[(1, 115200)]
        self.assertEqual([(r.id, r.status, r.R_us) for r in back], [(r.id, r.status, r.R_us) for r in recs])
        rows = core.write_summary_csv(d, core.load_all(d))
        self.assertEqual(rows[1]["ok"], 1)
        self.assertEqual(rows[0]["events"], 0)

    def test_baud_lines(self):
        f = core.parse_line("SCN,2,20,0,7,921600").fields
        self.assertEqual((f["scenario"], f["next_id"], f["baud"]), (2, 7, 921600))
        self.assertIsNone(core.parse_line("SCN,2,20,0,7").fields["baud"])     # eski firmware
        f = core.parse_line("BAUD,9600,115200").fields
        self.assertEqual((f["new"], f["old"]), (9600, 115200))

    def test_baud_csv_merge_and_legacy(self):
        d = tempfile.mkdtemp()
        # Eski (baud sutunsuz) dosya -> 115200 kabul edilir
        with open(core.scenario_csv_path(d, 5), "w", encoding="utf-8") as f:
            f.write("scenario,id,status,t0_us,t1_us,t2_us,t3_us,t4_us\n5,0,OK,0,10,12,100,900\n")
        data = core.load_all(d)
        self.assertEqual(list(data), [(5, 115200)])
        e = core.Experiment(5, 0, baud=9600)
        e.add(core.Record(5, 0, "OK", 0, 10, 12, 30000, 39400))
        e.add(core.Record(5, 2, "OK", 0, 10, 12, 100, 9500))
        data[(5, 9600)] = e.finalized()
        self.assertEqual([r.baud for r in data[(5, 9600)]], [9600, 9600, 9600])   # LOST_PC dahil
        core.write_scenario_csv(d, data, 5)
        again = core.load_all(d)
        self.assertEqual(sorted(again), [(5, 9600), (5, 115200)])
        self.assertEqual(len(again[(5, 9600)]), 3)
        rows = [r for r in core.summary_rows(again) if r["scenario"] == "S5"]
        self.assertEqual([(r["baud"], r["ok"]) for r in rows], [(9600, 2), (115200, 1)])
        self.assertEqual(len(core.summary_rows(again)), 7)                        # S0-S4 bos + S5 x2


if __name__ == "__main__":
    unittest.main()
