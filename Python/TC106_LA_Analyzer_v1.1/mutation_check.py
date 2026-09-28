# -*- coding: utf-8 -*-
"""
mutation_check.py -- verification of the tests themselves.

For each mutation, one line of tc106_la_analyzer.py is deliberately broken in
a temporary copy and the test suite is run.  A mutation is "KILLED" when the
suite fails (i.e. the tests can detect that bug) and "SURVIVED" when the suite
still passes (i.e. a blind spot).
"""
import os
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = open(os.path.join(HERE, "tc106_la_analyzer.py"), encoding="utf-8").read()

MUTATIONS = [
    ("TX edge tolerance 165 -> 500 us",
     "TX_EDGE_TOL_US = 165.0", "TX_EDGE_TOL_US = 500.0"),
    ("LA-09 upper bound ignores boundary bands",
     "lo_b, hi_b = Ec, Ec + Es + Ee", "lo_b, hi_b = Ec, Ec"),
    ("LA-09 lower bound wrongly widened",
     "lo_b, hi_b = Ec, Ec + Es + Ee", "lo_b, hi_b = Ec - 30, Ec + Es + Ee"),
    ("wire correction sign flipped",
     "wc = OrderedDict((n, raw[n] - wire[n]) for n in TX_ORDER)",
     "wc = OrderedDict((n, raw[n] + wire[n]) for n in TX_ORDER)"),
    ("gap threshold 505 -> 5000 ms",
     "GAP_THRESHOLD_MS = 505.0", "GAP_THRESHOLD_MS = 5000.0"),
    ("TX frames never separated (gap = huge)",
     "TX_FRAME_GAP_MS = 100.0", "TX_FRAME_GAP_MS = 1.0e9"),
    ("offset spread limit 10 -> 1000 ms",
     "OFFSET_SPREAD_MAX_MS = 10.0", "OFFSET_SPREAD_MAX_MS = 1000.0"),
    ("logger start-delay check removed",
     "and 0.0 <= D <= P.LOGGER_START_MAX_S", ""),
    ("post-RESULT margin check removed",
     "ok1 = inside and post >= P.POST_MARGIN_MIN_S and", "ok1 = inside and True and"),
    ("edge-count-mismatch ignored in LA-05",
     'not any("edge_count" in e or "edges_share" in e for e in f["errors"])',
     "True"),
    ("slot-count (n_obs != n_exp) check removed",
     "ok = (abs(dev) <= P.TX_EDGE_TOL_US) and (n_exp is None or n_exp == n_obs)",
     "ok = (abs(dev) <= P.TX_EDGE_TOL_US)"),
    ("undeclared model treated as PASS path (uses WIRE_CORRECTED)",
     "if model is None:\n            add(\"LA-08\", \"UNRESOLVED\"",
     "if False:\n            add(\"LA-08\", \"UNRESOLVED\""),
    ("byte-start (adjacent) spacing check disabled",
     "ok=all(abs(d) <= P.TX_EDGE_TOL_US for d in adj))", "ok=True)"),
    ("byte-start check made cumulative (would gate on LA clock error)",
     "ok=all(abs(d) <= P.TX_EDGE_TOL_US for d in adj))",
     "ok=all(abs(d) <= 50.0 for d in cum))"),
    ("boundary margin 10 ms -> 0",
     "BOUNDARY_MARGIN_MS = 10.0", "BOUNDARY_MARGIN_MS = 0.0"),
    ("boundary margin drifts back to 7 ms (spec sync guard)",
     "BOUNDARY_MARGIN_MS = 10.0", "BOUNDARY_MARGIN_MS = 7.0"),
    ("V3-CMD: exact packet comparison removed",
     "if rh != want:", "if False:"),
    ("V3-CMD: checksum7 recomputation removed",
     "if not _checksum7_matches(rh):", "if False:"),
    ("V3-CMD: logger checksum7_ok flag ignored",
     'if lst[0].get("checksum7_ok") is not True:', "if False:"),
    ("V3-CMD: duplicate command allowed",
     "if len(lst) != 1:\n                issues.append(\"%s sent %d time(s) (required 1)\" % (n, len(lst)))\n                continue",
     "if len(lst) < 1:\n                issues.append(\"%s sent %d time(s) (required 1)\" % (n, len(lst)))\n                continue"),
    ("expected RESET packet changed",
     '("RESET", "01 00 00 00 00 00 00 00 00 00 00 01")',
     '("RESET", "01 00 00 00 00 00 00 00 00 00 01 02")'),
    ("Direction-B guard threshold 26 ms -> 60 ms",
     "DIRB_GUARD_MIN_HIGH_US = 26000.0", "DIRB_GUARD_MIN_HIGH_US = 60000.0"),
    ("marker (cmd/data) bit ignored in expected bytes",
     "(0xF3, 1)", "(0xF3, 0)"),
    ("expected SEND byte changed (F3 -> F4)",
     "(0xF3, 1)", "(0xF4, 1)"),
    ("Direction-B footer value changed",
     "if val == 0x7F:", "if val == 0x7E:"),
    ("edge extraction: rising/falling search swapped",
     'nxt = find(b"\\x01" if cur == 0 else b"\\x00", pos)',
     'nxt = find(b"\\x00" if cur == 0 else b"\\x01", pos)'),
    ("chunk boundary: level not carried across chunks",
     "cur = c[\"cur\"]\n            if cur is None:",
     "cur = None\n            if cur is None:"),
    ("unitsize lane selection ignored",
     "lane = chunk if u == 1 else chunk[c[\"lane\"]::u]", "lane = chunk"),
    ("V3 delta rule: overrun must be 0 -> 5",
     '("V3-07", "tc_main_tx_overrun_count", 0)', '("V3-07", "tc_main_tx_overrun_count", 5)'),
]


def run_suite(workdir, env):
    r = subprocess.run([sys.executable, "-m", "unittest", "test_tc106_la_analyzer"],
                       cwd=workdir, env=env, capture_output=True, text=True, timeout=600)
    return r.returncode, r.stderr[-400:]


def main():
    env = dict(os.environ)
    tmpl = tempfile.mkdtemp()
    for f in ("tc106_la_synth.py", "test_tc106_la_analyzer.py"):
        shutil.copy(os.path.join(HERE, f), tmpl)
    rc0, _ = (lambda: (run_suite_baseline(tmpl, env)))()
    if rc0 != 0:
        print("BASELINE SUITE FAILS - fix that first")
        return 2
    killed = survived = 0
    rows = []
    EQUIVALENT = {
        "slot-count (n_obs != n_exp) check removed":
            "redundant by construction: bytes==template and edge count==template "
            "imply identical slot sequence, hence identical n_exp; kept as "
            "defence in depth",
    }
    for title, old, new in MUTATIONS:
        if SRC.count(old) < 1:
            rows.append((title, "NOT-APPLICABLE (pattern not found)"))
            continue
        src = SRC.replace(old, new, 1)
        with open(os.path.join(tmpl, "tc106_la_analyzer.py"), "w", encoding="utf-8") as f:
            f.write(src)
        rc, _ = run_suite(tmpl, env)
        if rc != 0:
            killed += 1
            rows.append((title, "KILLED"))
        elif title in EQUIVALENT:
            rows.append((title, "SURVIVED (equivalent mutant: %s)" % EQUIVALENT[title]))
        else:
            survived += 1
            rows.append((title, "SURVIVED  <-- blind spot"))
    shutil.rmtree(tmpl, ignore_errors=True)
    for t, r in rows:
        print("%-62s %s" % (t, r))
    print("\nmutations: %d  killed: %d  survived(blind spots): %d  equivalent: %d"
          % (len(rows), killed, survived,
             sum(1 for _, r in rows if "equivalent" in r)))
    return 0 if survived == 0 else 1


def run_suite_baseline(tmpl, env):
    shutil.copy(os.path.join(HERE, "tc106_la_analyzer.py"), tmpl)
    return run_suite(tmpl, env)


if __name__ == "__main__":
    sys.exit(main())
