# -*- coding: utf-8 -*-
"""
Tests for tc106_la_analyzer.py.

  python3 -m unittest -v test_tc106_la_analyzer            (fast: unit + small .sr)
  TC106_PREFLIGHT_SR=/path/preflight.sr  python3 -m unittest ...   (real file)
  TC106_FULLSIZE=1                       python3 -m unittest ...   (150 s capture)
"""
import contextlib
import io
import json
import os
import shutil
import sys
import tempfile
import time
import unittest

import tc106_la_analyzer as A
import tc106_la_synth as S

P = A.Params()


def edgelists(sc, fs=1e6):
    def el(edges):
        return A.EdgeList.from_times_us([t for t, _ in edges], [l for _, l in edges],
                                        1, sc["capture_s"], fs)
    return el(sc["tx"]), el(sc["rx"])


def run(sc, model="WIRE_CORRECTED", formal=False):
    tx, rx = edgelists(sc)
    cap = dict(fs=1e6, duration_s=sc["capture_s"])
    return A.analyze_edges(cap, tx, rx, sc["evidence"], model, P, formal)


def st(res, i):
    return (res["results_la"].get(i) or res["results_v3"].get(i))["status"]


class TestTemplates(unittest.TestCase):
    """Cross-check of the expected-frame templates against numbers derived
    independently from the TcMainTx slot logic (earlier simulation)."""

    def test_last_visible_edge_and_edge_counts(self):
        want = {"SEND": (141.9, 6), "RESET": (485.1, 14), "SENS.ADJ": (16.5, 2)}
        for name, (last_ms, falling) in want.items():
            e = A.template_expected_edges(name)
            self.assertAlmostEqual(e[-1][0] / 1e3, last_ms, places=3, msg=name)
            self.assertEqual(sum(1 for _, l in e if l == 0), falling, msg=name)

    def test_nominal_lengths(self):
        for name, ms in (("SEND", 171.6), ("RESET", 514.8), ("SENS.ADJ", 36.3)):
            n = len(A.template_slot_levels(A.TX_TEMPLATES[name]))
            self.assertAlmostEqual(n * 3.3, ms, places=6, msg=name)

    def test_expected_bytes_are_the_specified_ones(self):
        b = lambda n: " ".join("%02X" % v for v, _ in A.TX_TEMPLATES[n]["bytes"])
        self.assertEqual(b("SEND"), "00 F3 00 FB")
        self.assertEqual(b("RESET"), "00 00 00 00 00 F1 00 00 00 00 00 F9")
        self.assertEqual(b("SENS.ADJ"), "F2")


class TestGoodRuns(unittest.TestCase):
    def test_good_ts_at_write_start(self):
        r = run(S.build_scenario(pi_ts_at="start"), "WIRE_CORRECTED")
        self.assertEqual(r["overall"], "ALL_AUTOMATED_ITEMS_PASS", json.dumps(
            {k: v["status"] for k, v in list(r["results_la"].items())}, indent=1))
        self.assertEqual(st(r, "LA-10"), "MANUAL")

    def test_good_ts_after_flush_wire_elapsed(self):
        r = run(S.build_scenario(pi_ts_at="end"), "RAW")
        self.assertEqual(r["overall"], "ALL_AUTOMATED_ITEMS_PASS")

    def test_wrong_model_shows_larger_spread_but_is_reported(self):
        sc = S.build_scenario(pi_ts_at="end")
        r = run(sc, "WIRE_CORRECTED")
        sp = r["results_la"]["LA-08"]["detail"]
        self.assertIn("raw spread", sp)
        # the wire-corrected spread is ~6.25 ms here: still <= 10 ms, so the
        # tool cannot tell models apart from the limit alone -> both reported.
        self.assertIn("wire-corrected spread", sp)

    def test_edge_interval_rule_boundary(self):
        r = run(S.build_scenario(faults=["late_edge_120"]))
        self.assertEqual(st(r, "LA-11"), "PASS")          # 120 us < 165 us


class TestInjectedFaults(unittest.TestCase):
    def check(self, fault, expect):
        r = run(S.build_scenario(faults=[fault]))
        for i, want in expect.items():
            self.assertEqual(st(r, i), want, "%s -> %s: %s" % (
                fault, i, (r["results_la"].get(i) or r["results_v3"].get(i))["detail"]))
        self.assertEqual(r["overall"], "HAS_FAIL", fault)
        return r

    def test_late_edge_300us(self):
        self.check("late_edge_300", {"LA-11": "FAIL"})

    def test_truncated_reset(self):
        r = self.check("trunc_reset", {"LA-04": "FAIL", "LA-05": "FAIL", "LA-06": "FAIL"})
        unk = r["results_la"]["LA-04"]["unknown"][0]
        self.assertIn("spb13", unk["decode_diag"])

    def test_wrong_byte(self):
        self.check("wrong_byte", {"LA-04": "FAIL", "LA-06": "FAIL"})

    def test_missing_sens_adj(self):
        self.check("missing_sens", {"LA-04": "FAIL"})

    def test_extra_tx_frame(self):
        self.check("extra_frame", {"LA-04": "FAIL"})

    def test_glitch_inside_frame_is_caught_by_edge_count(self):
        # byte values still decode correctly; only the edge structure differs
        r = self.check("glitch_in_frame", {"LA-05": "FAIL"})
        self.assertIn(st(r, "LA-04"), ("PASS", "FAIL"))

    def test_short_glitch_on_tx_line(self):
        self.check("glitch", {"LA-04": "FAIL"})

    def test_rx_gap_700ms(self):
        self.check("rx_gap", {"LA-07": "FAIL"})

    def test_esp_counter_too_high(self):
        r = self.check("c1_high", {"LA-09": "FAIL"})
        self.assertGreater(r["results_la"]["LA-09"]["E_ESP"],
                           r["results_la"]["LA-09"]["E_LA_max"])

    def test_esp_counter_too_low(self):
        self.check("c1_low", {"LA-09": "FAIL"})

    def test_offset_spread_over_10ms(self):
        self.check("spread_big", {"LA-08": "FAIL"})

    def test_capture_too_short_for_post_margin(self):
        sc = S.build_scenario(capture_s=31.0)      # RESULT at 28.23 -> post 2.8 s
        r = run(sc)
        self.assertEqual(st(r, "LA-01"), "FAIL")

    def test_logger_started_too_late(self):
        sc = S.build_scenario(D=19.5, capture_s=60.0)
        r = run(sc)
        self.assertEqual(st(r, "LA-01"), "FAIL")


class TestUnresolvedNotPass(unittest.TestCase):
    def test_model_not_declared(self):
        r = run(S.build_scenario(), model=None)
        for i in ("LA-01", "LA-07", "LA-08", "LA-09"):
            self.assertEqual(st(r, i), "UNRESOLVED", i)
        self.assertEqual(r["overall"], "HAS_UNRESOLVED")

    def test_timeout_json_without_evidence(self):
        r = run(S.build_scenario(faults=["no_evidence"]))
        self.assertEqual(st(r, "V3-01"), "UNRESOLVED")
        self.assertEqual(st(r, "LA-09"), "UNRESOLVED")
        self.assertNotEqual(r["overall"], "ALL_AUTOMATED_ITEMS_PASS")

    def test_formal_requires_120s(self):
        r = run(S.build_scenario(), formal=True)     # synthetic window is 20 s
        self.assertEqual(st(r, "V3-01"), "FAIL")


class TestBoundaryBands(unittest.TestCase):
    """LA-09 tolerance must actually be exercised: the scenario phase is swept
    until edges fall inside BOTH boundary bands."""

    @staticmethod
    def find_phase_with_band_edges():
        for ph in range(0, 60000, 500):                       # 0..60 ms step 0.5
            sc = S.build_scenario(rx_begin_us=0.02e6 + ph)
            d = run(sc)["results_la"]["LA-09"]
            if d["E_start"] > 0 and d["E_end"] > 0:
                return sc, d
        return None, None

    def test_a_phase_with_edges_in_both_bands_exists(self):
        sc, d = self.find_phase_with_band_edges()
        self.assertIsNotNone(sc, "no phase puts edges into both boundary bands")
        self.assertGreater(d["E_LA_max"], d["E_LA_min"])

    def test_tolerance_is_inclusive_and_one_more_fails(self):
        sc, d = self.find_phase_with_band_edges()
        lo, hi = d["E_LA_min"], d["E_LA_max"]
        for target, want in ((sc["e_esp"], "PASS"), (lo, "PASS"), (hi, "PASS"),
                             (hi + 1, "FAIL"), (lo - 1, "FAIL")):
            sc2 = copy_scenario(sc)
            c = sc2["evidence"]["counters"]["c1_isr_edges"]
            c["end"] = c["start"] + target
            c["delta"] = target
            self.assertEqual(st(run(sc2), "LA-09"), want, "E_ESP=%d [%d,%d]" % (target, lo, hi))

    def test_truth_is_always_inside_the_band_for_every_phase(self):
        """Property: the true ESP count never falls outside [E_min, E_max] when
        the true window start is inside the declared +-10 ms margin."""
        bad = []
        for ph in range(0, 60000, 1000):
            for seed in (1, 2, 3):
                sc = S.build_scenario(seed=seed, rx_begin_us=0.02e6 + ph)
                if st(run(sc), "LA-09") != "PASS":
                    bad.append((ph, seed))
        self.assertEqual(bad, [])


def copy_scenario(sc):
    import copy
    sc2 = dict(sc)
    sc2["evidence"] = copy.deepcopy(sc["evidence"])
    return sc2


class TestMarginPinnedToSpec(unittest.TestCase):
    """Boundary margin is +-10 ms (review agreement, B section 4.6.5).  These
    tests fail if the constant and the report/window arithmetic drift apart
    from the specification again."""

    def test_constant_is_10_ms(self):
        self.assertEqual(A.Params.BOUNDARY_MARGIN_MS, 10.0)

    def test_band_width_is_offset_spread_plus_two_margins(self):
        r = run(S.build_scenario())
        w = r["window_la_s"]
        offs = list(r["results_la"]["LA-08"]["offsets_ms"]["WIRE_CORRECTED"].values())
        spread_s = (max(offs) - min(offs)) / 1e3
        self.assertAlmostEqual(w["start_hi_s"] - w["start_lo_s"], spread_s + 0.020, places=6)
        self.assertAlmostEqual(w["end_hi_s"] - w["end_lo_s"], spread_s + 0.020, places=6)

    def test_end_band_is_start_band_shifted_by_duration(self):
        r = run(S.build_scenario())
        w = r["window_la_s"]
        self.assertAlmostEqual(w["end_lo_s"] - w["start_lo_s"], 20.0, places=6)

    def test_report_records_the_margin(self):
        self.assertEqual(A.Params().as_dict()["BOUNDARY_MARGIN_MS"], 10.0)


class TestPiCommandPackets(unittest.TestCase):
    """V3-CMD: exact Pi packets + checksum7 (recomputed, not just trusted)."""

    def test_good_run_passes(self):
        self.assertEqual(st(run(S.build_scenario()), "V3-CMD"), "PASS")

    def test_nonzero_data_with_valid_checksum_fails(self):
        r = run(S.build_scenario(faults=["cmd_nonzero"]))
        self.assertEqual(st(r, "V3-CMD"), "FAIL")
        self.assertEqual(r["overall"], "HAS_FAIL")

    def test_bad_checksum_fails(self):
        self.assertEqual(st(run(S.build_scenario(faults=["cmd_checksum_bad"])), "V3-CMD"), "FAIL")

    def test_logger_flag_lying_is_caught_by_recomputation(self):
        r = run(S.build_scenario(faults=["cmd_flag_lies"]))
        self.assertEqual(st(r, "V3-CMD"), "FAIL")
        self.assertIn("recomputation", r["results_v3"]["V3-CMD"]["detail"])

    def test_logger_self_check_false_while_bytes_look_right_fails(self):
        r = run(S.build_scenario(faults=["cmd_flag_false_hex_ok"]))
        self.assertEqual(st(r, "V3-CMD"), "FAIL")
        self.assertIn("checksum7_ok", r["results_v3"]["V3-CMD"]["detail"])
        self.assertNotIn("recomputation", r["results_v3"]["V3-CMD"]["detail"])

    def test_duplicate_send_fails(self):
        self.assertEqual(st(run(S.build_scenario(faults=["cmd_duplicate"])), "V3-CMD"), "FAIL")

    def test_missing_scripted_commands_is_unresolved(self):
        sc = S.build_scenario()
        sc["evidence"]["scripted_commands"] = []
        self.assertEqual(st(run(sc), "V3-CMD"), "UNRESOLVED")

    def test_case_and_spacing_are_normalised(self):
        sc = S.build_scenario()
        for c in sc["evidence"]["scripted_commands"]:
            c["raw_hex"] = "  " + c["raw_hex"].lower().replace(" ", "  ") + " "
        self.assertEqual(st(run(sc), "V3-CMD"), "PASS")


@unittest.skipUnless(os.environ.get("TC106_REAL_EVIDENCE_JSON") and
                     os.path.exists(os.environ.get("TC106_REAL_EVIDENCE_JSON", "")),
                     "set TC106_REAL_EVIDENCE_JSON to the real 9/27 PASS-run JSON")
class TestRealEvidenceJson(unittest.TestCase):
    def test_real_pass_run_json_is_all_v3_pass(self):
        ev = json.load(open(os.environ["TC106_REAL_EVIDENCE_JSON"], encoding="utf-8"))
        v = A.evaluate_v3(ev, P, True)
        bad = {k: x["status"] for k, x in v.items()
               if x["status"] not in ("PASS", "AUX_PASS")}
        self.assertEqual(bad, {})
        self.assertIn("V3-CMD", v)


class TestClockAndDrift(unittest.TestCase):
    def test_edge_intervals_ok_but_byte_start_spacing_off_is_caught(self):
        r = run(S.build_scenario(faults=["drift_within_frame"]))
        self.assertEqual(st(r, "LA-11"), "PASS")          # every edge interval <= 60 us
        self.assertEqual(st(r, "LA-06"), "FAIL")          # adjacent byte start drifted 240 us
        self.assertEqual(r["overall"], "HAS_FAIL")

    def test_la_clock_error_of_200ppm_does_not_fail_a_good_run(self):
        """LA sample clock 200 ppm off: all times scale.  Adjacent byte-start
        spacing changes by 8.6 us (fine); the informational cumulative
        deviation over RESET (~0.5 s) reaches ~100 us and must NOT gate."""
        sc = S.build_scenario()
        k = 1.0 + 200e-6
        sc["tx"] = [(t * k, l) for t, l in sc["tx"]]
        sc["rx"] = [(t * k, l) for t, l in sc["rx"]]
        r = run(sc)
        self.assertEqual(st(r, "LA-06"), "PASS")
        self.assertEqual(st(r, "LA-11"), "PASS")
        reset = [f for f in r["tx"]["frames"] if f["name"] == "RESET"][0]
        self.assertGreater(reset["byte_start"]["cumulative_max_abs_dev_us"], 50.0)
        self.assertLess(reset["byte_start"]["adjacent_max_abs_dev_us"], 20.0)


class TestTxFrameDecoder(unittest.TestCase):
    def test_all_three_frames_identified_with_exact_bytes(self):
        r = run(S.build_scenario())
        names = [f["name"] for f in r["tx"]["frames"]]
        self.assertEqual(names, ["SEND", "RESET", "SENS.ADJ"])
        self.assertEqual(r["tx"]["frames"][1]["decoded_bytes"],
                         "00 00 00 00 00 F1 00 00 00 00 00 F9")

    def test_byte_start_spacing_is_42_9ms(self):
        r = run(S.build_scenario())
        bs = r["tx"]["frames"][0]["byte_start"]
        self.assertAlmostEqual(bs["expected_spacing_us"], 42900.0)
        self.assertTrue(bs["ok"])

    def test_lever_full_frame_length_is_not_used(self):
        """last visible edge is far shorter than the nominal length; the tool
        must still PASS (regression for the v1.2 review finding)."""
        r = run(S.build_scenario())
        self.assertEqual(st(r, "LA-05"), "PASS")


class TestSrContainer(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def _write(self, name, edges_by_ch, dur=3.0, unitsize=1, chunk=1 << 20):
        p = os.path.join(self.tmp, name)
        S.write_sr(p, dur, edges_by_ch, unitsize=unitsize, chunk_samples=chunk)
        return p

    def test_edge_exactly_on_chunk_boundary(self):
        chunk = 1 << 20                           # 1,048,576 samples
        ch = {"D0": dict(bit=0, initial=1, edges=[(chunk - 1, 0), (chunk, 1),
                                                    (2 * chunk, 0), (2 * chunk + 5, 1)])}
        sr = A.SrFile(self._write("b.sr", ch, dur=3.0, chunk=chunk))
        e = A.extract_edges(sr, ["D0"])["D0"]
        self.assertEqual(e.idx, [chunk - 1, chunk, 2 * chunk, 2 * chunk + 5])
        self.assertEqual(e.lvl, [0, 1, 0, 1])

    def test_level_continues_across_chunks_without_false_edge(self):
        chunk = 1 << 20
        ch = {"D0": dict(bit=0, initial=1, edges=[(10, 0), (3 * chunk - 10, 1)])}
        sr = A.SrFile(self._write("c.sr", ch, dur=3.5, chunk=chunk))
        e = A.extract_edges(sr, ["D0"])["D0"]
        self.assertEqual(e.idx, [10, 3 * chunk - 10])

    def test_unitsize_2_high_channel(self):
        ch = {"D0": dict(bit=0, initial=1, edges=[(100, 0), (200, 1)]),
              "D9": dict(bit=9, initial=1, edges=[(150, 0), (400, 1)])}
        sr = A.SrFile(self._write("u.sr", ch, dur=0.01, unitsize=2, chunk=4096))
        e = A.extract_edges(sr, ["D0", "D9"])
        self.assertEqual(e["D0"].idx, [100, 200])
        self.assertEqual(e["D9"].idx, [150, 400])

    def test_unknown_channel_is_an_error(self):
        ch = {"D0": dict(bit=0, initial=1, edges=[(10, 0)])}
        sr = A.SrFile(self._write("e.sr", ch, dur=0.01))
        with self.assertRaises(ValueError):
            A.extract_edges(sr, ["D5"])

    def test_samplerate_parsing(self):
        for s, v in (("1 MHz", 1e6), ("500 kHz", 5e5), ("24 MHz", 24e6),
                     ("1000000", 1e6), ("100 kHz", 1e5)):
            self.assertEqual(A.parse_samplerate(s), v)


class TestEndToEndCLI(unittest.TestCase):
    """Small real .sr files through the command line."""

    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.mkdtemp()

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(cls.tmp, ignore_errors=True)

    def _make(self, tag, **kw):
        sc = S.build_scenario(**kw)
        srp = os.path.join(self.tmp, tag + ".sr")
        S.write_sr(srp, sc["capture_s"],
                   {"D0": dict(bit=0, initial=1, edges=sc["tx"]),
                    "D1": dict(bit=1, initial=1, edges=sc["rx"])})
        jp = os.path.join(self.tmp, tag + ".json")
        with open(jp, "w") as f:
            json.dump(sc["evidence"], f)
        return srp, jp

    def _cli(self, args):
        out, err = io.StringIO(), io.StringIO()
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
            try:
                code = A.main(args)
            except SystemExit as e:
                code = e.code
        return code, out.getvalue(), err.getvalue()

    def test_good_run_exit_0(self):
        srp, jp = self._make("good")
        outj = os.path.join(self.tmp, "good_report.json")
        code, out, _ = self._cli(["analyze", "--sr", srp, "--evidence-json", jp,
                                  "--tx", "D0", "--rx", "D1",
                                  "--offset-model", "WIRE_CORRECTED", "--out-json", outj])
        self.assertEqual(code, 0, out)
        rep = json.load(open(outj))
        self.assertEqual(rep["overall"], "ALL_AUTOMATED_ITEMS_PASS")
        self.assertEqual(rep["tool"]["sha256"], A.tool_sha256())
        self.assertIn("TX_EDGE_TOL_US", rep["parameters"])

    def test_truncated_frame_exit_1(self):
        srp, jp = self._make("trunc", faults=["trunc_reset"])
        code, out, _ = self._cli(["analyze", "--sr", srp, "--evidence-json", jp,
                                  "--tx", "D0", "--rx", "D1",
                                  "--offset-model", "WIRE_CORRECTED"])
        self.assertEqual(code, 1)
        self.assertIn("LA-05", out)

    def test_swapped_channels_do_not_pass(self):
        srp, jp = self._make("swap")
        code, out, _ = self._cli(["analyze", "--sr", srp, "--evidence-json", jp,
                                  "--tx", "D1", "--rx", "D0",
                                  "--offset-model", "WIRE_CORRECTED"])
        self.assertNotEqual(code, 0)

    def test_undeclared_model_exit_2(self):
        srp, jp = self._make("nomodel")
        code, out, _ = self._cli(["analyze", "--sr", srp, "--evidence-json", jp,
                                  "--tx", "D0", "--rx", "D1"])
        self.assertEqual(code, 2)

    def test_formal_without_model_is_usage_error(self):
        srp, jp = self._make("formalusage")
        code, _, err = self._cli(["analyze", "--sr", srp, "--evidence-json", jp,
                                  "--tx", "D0", "--rx", "D1", "--formal"])
        self.assertEqual(code, 3)

    def test_tx_probed_at_gate_needs_invert_flag(self):
        sc = S.build_scenario()
        srp = os.path.join(self.tmp, "gate.sr")
        inv = [(t, 1 - l) for t, l in sc["tx"]]
        S.write_sr(srp, sc["capture_s"], {"D0": dict(bit=0, initial=0, edges=inv),
                                          "D1": dict(bit=1, initial=1, edges=sc["rx"])})
        jp = os.path.join(self.tmp, "gate.json")
        with open(jp, "w") as f:
            json.dump(sc["evidence"], f)
        base = ["analyze", "--sr", srp, "--evidence-json", jp, "--tx", "D0",
                "--rx", "D1", "--offset-model", "WIRE_CORRECTED"]
        self.assertEqual(self._cli(base)[0], 1)                 # wrong polarity -> FAIL
        self.assertEqual(self._cli(base + ["--tx-invert"])[0], 0)

    def test_broken_inputs_are_exit_3_not_1(self):
        bad = os.path.join(self.tmp, "notsr.sr")
        with open(bad, "w") as f:
            f.write("garbage")
        jp = os.path.join(self.tmp, "x.json")
        with open(jp, "w") as f:
            f.write("{}")
        code, _, err = self._cli(["analyze", "--sr", bad, "--evidence-json", jp,
                                  "--tx", "D0", "--rx", "D1"])
        self.assertEqual(code, 3)
        self.assertIn("INPUT ERROR", err)
        code, _, err = self._cli(["analyze", "--sr", "/nonexistent.sr",
                                  "--evidence-json", jp, "--tx", "D0", "--rx", "D1"])
        self.assertEqual(code, 3)

    def test_unknown_channel_name_is_exit_3(self):
        srp, jp = self._make("chan")
        code, _, err = self._cli(["analyze", "--sr", srp, "--evidence-json", jp,
                                  "--tx", "D5", "--rx", "D1"])
        self.assertEqual(code, 3)

    def test_refuses_to_overwrite_report(self):
        srp, jp = self._make("ow")
        outj = os.path.join(self.tmp, "ow_report.json")
        open(outj, "w").write("keep me")
        code, _, err = self._cli(["analyze", "--sr", srp, "--evidence-json", jp,
                                  "--tx", "D0", "--rx", "D1",
                                  "--offset-model", "WIRE_CORRECTED", "--out-json", outj])
        self.assertEqual(code, 3)
        self.assertEqual(open(outj).read(), "keep me")


@unittest.skipUnless(os.environ.get("TC106_PREFLIGHT_SR") and
                     os.path.exists(os.environ.get("TC106_PREFLIGHT_SR", "")),
                     "set TC106_PREFLIGHT_SR to the real preflight .sr")
class TestRealPreflightFile(unittest.TestCase):
    """Numbers stated in TC106_PulseView_SR_Preflight_Report_20260928.md"""

    @classmethod
    def setUpClass(cls):
        cls.sr = A.SrFile(os.environ["TC106_PREFLIGHT_SR"])
        cls.e = A.extract_edges(cls.sr, ["D0", "D1"])

    def test_container(self):
        self.assertEqual(self.sr.sigrok_version, "0.5.2")
        self.assertEqual(self.sr.samplerate_hz, 1e6)
        self.assertEqual(self.sr.total_probes, 8)
        self.assertEqual(self.sr.unitsize, 1)
        self.assertEqual(len(self.sr.chunk_names), 20)
        self.assertEqual(self.sr.n_samples, 200000000)
        self.assertEqual(self.sr.sha256(),
                         "592c0cd6cbb86589659b8cc4a8ce3c3034d5a39e4b390aa5980adc92d8940dbb")

    def test_edges(self):
        for ch in ("D0", "D1"):
            s = A.channel_stats(self.e[ch])
            self.assertEqual((s["edges"], s["rising"], s["falling"]), (7142, 3571, 3571))
            self.assertEqual(s["first_edge_ms"], 20.072)
            self.assertEqual(s["last_edge_s"], 199.998627)
            self.assertEqual(s["max_edge_interval_ms"], 52.744)
            self.assertEqual(s["min_edge_interval_ms"], 3.275)

    def test_d0_d1_delta_histogram(self):
        from collections import Counter
        h = Counter(b - a for a, b in zip(self.e["D0"].idx, self.e["D1"].idx))
        self.assertEqual(dict(h), {0: 6611, 1: 531})

    def test_independent_direction_b_decoder_on_real_nano_waveform(self):
        d = A.decode_dirb(self.e["D1"].t_us, self.e["D1"].lvl,
                          self.e["D1"].initial_level, P)
        self.assertEqual(len(d["frames"]), 594)
        self.assertTrue(all(f["payload"] == [0, 0, 0, 0, 0] for f in d["frames"]))
        self.assertEqual(d["sync_misses"], 0)
        self.assertEqual(d["interval_warn"], 0)


@unittest.skipUnless(os.environ.get("TC106_FULLSIZE"), "set TC106_FULLSIZE=1")
class TestFullSize(unittest.TestCase):
    def test_150s_capture_end_to_end(self):
        tmp = tempfile.mkdtemp()
        try:
            sc = S.build_scenario(capture_s=150.0, window_s=120.0, D=10.0,
                                  cmd_sched=(20.0, 50.0, 80.0))
            srp = os.path.join(tmp, "full.sr")
            t0 = time.time()
            S.write_sr(srp, 150.0, {"D0": dict(bit=0, initial=1, edges=sc["tx"]),
                                    "D1": dict(bit=1, initial=1, edges=sc["rx"])})
            gen = time.time() - t0
            jp = os.path.join(tmp, "full.json")
            json.dump(sc["evidence"], open(jp, "w"))
            t0 = time.time()
            out = io.StringIO()
            with contextlib.redirect_stdout(out):
                code = A.main(["analyze", "--sr", srp, "--evidence-json", jp,
                               "--tx", "D0", "--rx", "D1",
                               "--offset-model", "WIRE_CORRECTED", "--formal"])
            took = time.time() - t0
            sys.stderr.write("\n[fullsize] generate %.1fs, analyze %.1fs, exit=%s, "
                             "samples=150,000,000\n" % (gen, took, code))
            self.assertEqual(code, 0, out.getvalue())
            self.assertLess(took, 60.0)
        finally:
            shutil.rmtree(tmp, ignore_errors=True)


if __name__ == "__main__":
    unittest.main(verbosity=2)
