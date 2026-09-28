# -*- coding: utf-8 -*-
"""
tc106_la_synth.py -- synthetic waveforms and sigrok .sr writer, for testing
tc106_la_analyzer.py.  Test support only; NOT part of the Formal tool hash.
"""
import bisect
import copy
import random
import zipfile

from tc106_la_analyzer import TX_TEMPLATES, template_slot_levels, Params

SLOT = Params.SLOT_US
UART_BITS, BAUD = 10.0, 9600.0

COUNTER_NAMES = [
    "poll_max_interval_us", "poll_over1000_count",
    "poll_busy_max_interval_us_window", "poll_busy_over1000_count_window",
    "poll_idle_max_interval_us_window", "poll_idle_over1000_count_window",
    "tc_main_tx_overrun_count", "tx_reset_started_count", "tx_send_started_count",
    "tx_sens_started_count", "tx_done_count", "tx_aborted_done_count",
    "c1_isr_edges", "c1_edge_overflow", "c2_decoded_bytes", "c2_decode_errors",
    "c2_timing_errors", "c2_local_edge_overflow", "c3_valid_frames",
    "c3_frame_errors", "c3_sync_misses", "c3_resyncs", "c4_queued_frames",
    "c4_queue_drops"]


# ----------------------------------------------------------------------------
# waveform generators (return [(t_us, level_after_edge)])
# ----------------------------------------------------------------------------
def tx_frame_edges(name, t0_us, late_fn=None, mutate=None):
    lv = template_slot_levels(TX_TEMPLATES[name])
    if mutate:
        lv = mutate(lv)
    edges, prev = [], 1
    for s, l in enumerate(lv):
        if l != prev:
            k = len(edges)
            edges.append((t0_us + s * SLOT + (late_fn(k) if late_fn else 0.0), l))
            prev = l
    return edges


def dirb_edges(t_begin_us, t_end_us, payload=(0, 0, 0, 0, 0), slot_us=3290.0,
               drop=None):
    """Continuous Direction-B frames: per byte slot0 LOW, 7 data bits LSB first,
    then 9 guard slots HIGH (17 slots).  drop(frame_index)->True leaves the
    frame period idle HIGH (frame missing)."""
    edges, prev, t, fi = [], 1, t_begin_us, 0
    seq = list(payload) + [0x7F]
    while t < t_end_us:
        dropped = bool(drop and drop(fi))
        for v in seq:
            lv = [1] * 17 if dropped else \
                [0] + [(v >> b) & 1 for b in range(7)] + [1] * 9
            for s, l in enumerate(lv):
                if l != prev:
                    edges.append((t + s * slot_us, l))
                    prev = l
            t += 17 * slot_us
        fi += 1
    return [e for e in edges if e[0] <= t_end_us]


# ----------------------------------------------------------------------------
# sigrok .sr writer
# ----------------------------------------------------------------------------
def write_sr(path, duration_s, channels, fs=1e6, unitsize=1,
             chunk_samples=10 * 1024 * 1024, total_probes=None):
    """channels: {name: dict(bit=int, edges=[(t_us, lvl)], initial=1)}"""
    n_samples = int(round(duration_s * fs))
    scale = fs / 1e6
    events = {}
    cur_bits = 0
    for c in channels.values():
        if c.get("initial", 1):
            cur_bits |= 1 << c["bit"]
    per_ch = []
    for c in channels.values():
        for t, l in c["edges"]:
            per_ch.append((int(round(t * scale)), c["bit"], l))
    per_ch.sort()
    runs = [(0, cur_bits)]
    for idx, bit, l in per_ch:
        cur_bits = (cur_bits | (1 << bit)) if l else (cur_bits & ~(1 << bit))
        if idx <= runs[-1][0]:
            runs[-1] = (runs[-1][0], cur_bits)
        else:
            runs.append((idx, cur_bits))
    starts = [r[0] for r in runs]

    def render(a, b):
        parts = []
        i = bisect.bisect_right(starts, a) - 1
        pos = a
        while pos < b:
            nxt = starts[i + 1] if i + 1 < len(starts) else b
            end = min(b, nxt)
            v = runs[i][1]
            unit = v.to_bytes(unitsize, "little")
            parts.append(unit * (end - pos))
            pos = end
            i += 1
        return b"".join(parts)

    tp = total_probes or 8 * unitsize
    meta = ["[global]", "sigrok version=0.5.2", "", "[device 1]",
            "capturefile=logic-1", "total probes=%d" % tp,
            "samplerate=%s" % ("1 MHz" if fs == 1e6 else "%d Hz" % int(fs)),
            "total analog=0"]
    for name, c in channels.items():
        meta.append("probe%d=%s" % (c["bit"] + 1, name))
    meta += ["unitsize=%d" % unitsize, ""]
    with zipfile.ZipFile(path, "w") as z:
        z.writestr("version", "2")
        z.writestr("metadata", "\n".join(meta), compress_type=zipfile.ZIP_DEFLATED)
        k, a = 1, 0
        while a < n_samples:
            b = min(n_samples, a + chunk_samples)
            z.writestr("logic-1-%d" % k, render(a, b),
                       compress_type=zipfile.ZIP_DEFLATED, compresslevel=1)
            k += 1
            a = b


# ----------------------------------------------------------------------------
# Evidence V3 JSON (schema as produced by pi_logger_evidence_v3.py)
# ----------------------------------------------------------------------------
def make_evidence(window_s, cmd_pi_times, e_esp, evidence_received=True,
                  counters_override=None, max_interval_ms=351.4, frames=357):
    raws = {"SEND": "02 00 00 00 00 02",
            "RESET": "01 00 00 00 00 00 00 00 00 00 00 01",
            "SENS.ADJ": "03 00 00 00 00 00 00 03"}
    ctr = {}
    for n in COUNTER_NAMES:
        ctr[n] = dict(start=0, end=0, delta=0)
    ctr["tx_send_started_count"].update(end=1, delta=1)
    ctr["tx_reset_started_count"].update(end=1, delta=1)
    ctr["tx_sens_started_count"].update(end=1, delta=1)
    ctr["tx_done_count"].update(end=3, delta=3)
    ctr["c1_isr_edges"].update(start=3000, end=3000 + e_esp, delta=e_esp)
    ctr["poll_busy_max_interval_us_window"].update(end=18)
    for k, v in (counters_override or {}).items():
        ctr[k] = v
    ev = dict(magic_ok=True, version=2, version_ok=True, length=196, length_ok=True,
              checksum_ok=True, duration_ms=int(round(window_s * 1000)),
              duration_ok=True, counters=ctr,
              evidence_received=evidence_received,
              evidence_monotonic_s=window_s + 0.23,
              logger=dict(invalid_evidence_attempts=[]),
              scripted_commands=[dict(name=n, scheduled_s=t, sent_monotonic_s=t2,
                                      raw_hex=raws[n], checksum7_ok=True)
                                 for n, (t, t2) in cmd_pi_times.items()],
              normal_telemetry=dict(frames_received=frames, footer_ok_count=frames,
                                    max_interval_ms=max_interval_ms,
                                    boot_magic_frames=0, resync_drop_bytes=2,
                                    buffered_bytes_end=0))
    if not evidence_received:
        ev.pop("counters")
    return ev


# ----------------------------------------------------------------------------
# scenario builder
# ----------------------------------------------------------------------------
def build_scenario(seed=1, capture_s=40.0, window_s=20.0, D=8.0,
                   cmd_sched=(4.0, 9.0, 14.0), pi_ts_at="start",
                   lat_ms=(2.0, 3.5), faults=(), nano_slot_us=3290.0,
                   rx_begin_us=0.02e6):
    """LA time axis: t=0 is capture start.  Pi t=0 (first EVIDENCE_START) is at
    LA time D.  Returns dict(tx, rx, evidence, capture_s, ws, we)."""
    rnd = random.Random(seed)
    faults = set(faults)
    names = ["SEND", "RESET", "SENS.ADJ"]
    wire = {n: nb * UART_BITS / BAUD for n, nb in
            (("SEND", 6), ("RESET", 12), ("SENS.ADJ", 8))}
    tx, cmd_pi = [], OrderedDictLike()
    for k, n in enumerate(names):
        if n == "SENS.ADJ" and "missing_sens" in faults:
            cmd_pi[n] = (cmd_sched[k], cmd_sched[k] + (wire[n] if pi_ts_at == "end" else 0))
            continue
        lat = rnd.uniform(*lat_ms) / 1e3
        if "spread_big" in faults and n == "RESET":
            lat += 0.012
        t_pi_write = cmd_sched[k]
        la_start_us = (D + t_pi_write + wire[n] + lat) * 1e6
        late_fn = None
        mutate = None
        if n == "SEND" and "late_edge_300" in faults:
            late_fn = lambda i: 300.0 if i == 4 else 0.0
        if n == "SEND" and "late_edge_120" in faults:
            late_fn = lambda i: 120.0 if i == 4 else 0.0
        if n == "SEND" and "drift_within_frame" in faults:
            # every edge 60 us later than the previous one: each edge interval
            # is off by only 60 us (< 165) but byte starts drift by 240 us
            late_fn = lambda i: 60.0 * i
        if n == "SEND" and "wrong_byte" in faults:
            def mutate(lv):
                lv = list(lv)
                lv[3 * 13 + 1] = 0          # SEND byte3 0xFB -> 0xFA
                return lv
        if n == "RESET" and "trunc_reset" in faults:
            def mutate(lv):
                cut = 70
                return lv[:cut] + [1] * (len(lv) - cut)
        tx += tx_frame_edges(n, la_start_us, late_fn, mutate)
        if n == "SEND" and "glitch_in_frame" in faults:
            g = la_start_us + (11 * SLOT + 1500.0)     # inside HIGH guard slot 11 of byte0
            tx += [(g, 0), (g + 40.0, 1)]
        cmd_pi[n] = (t_pi_write, t_pi_write + (wire[n] if pi_ts_at == "end" else 0.0))
    if "extra_frame" in faults:
        tx += tx_frame_edges("SENS.ADJ", (D + 17.0) * 1e6)
    if "glitch" in faults:
        g = (D + 6.5) * 1e6
        tx += [(g, 0), (g + 50.0, 1)]
    tx.sort()
    ws = D + 4.17e-3 + 2.5e-3
    we = ws + window_s
    drop = None
    if "rx_gap" in faults:
        gap_lo = (D + window_s / 2) * 1e6
        drop = lambda fi: False        # replaced below (time based)
    rx = dirb_edges(rx_begin_us, capture_s * 1e6, slot_us=nano_slot_us)
    if "rx_gap" in faults:
        lo, hi = (D + window_s / 2) * 1e6, (D + window_s / 2 + 0.75) * 1e6
        # rebuild with the frames overlapping [lo, hi) removed
        period = 6 * 17 * nano_slot_us
        n0 = int((lo - rx_begin_us) // period)
        n1 = int((hi - rx_begin_us) // period) + 1
        rx = dirb_edges(rx_begin_us, capture_s * 1e6, slot_us=nano_slot_us,
                        drop=lambda fi: n0 <= fi <= n1)
    e_esp = sum(1 for t, _ in rx if ws * 1e6 <= t <= we * 1e6)
    if "c1_high" in faults:
        e_esp += 25
    if "c1_low" in faults:
        e_esp -= 25
    ev = make_evidence(window_s, cmd_pi, e_esp,
                       evidence_received=("no_evidence" not in faults))
    sc_list = ev["scripted_commands"]
    if "cmd_nonzero" in faults:                    # valid checksum, but data != 0
        for c in sc_list:
            if c["name"] == "SEND":
                c["raw_hex"] = "02 05 00 00 00 07"
    if "cmd_checksum_bad" in faults:               # wrong checksum, logger admits it
        for c in sc_list:
            if c["name"] == "RESET":
                c["raw_hex"] = c["raw_hex"][:-2] + "05"
                c["checksum7_ok"] = False
    if "cmd_flag_lies" in faults:                  # wrong checksum but flag says OK
        for c in sc_list:
            if c["name"] == "SENS.ADJ":
                c["raw_hex"] = c["raw_hex"][:-2] + "04"
    if "cmd_flag_false_hex_ok" in faults:          # bytes are right, logger says NG
        for c in sc_list:
            if c["name"] == "SEND":
                c["checksum7_ok"] = False
    if "cmd_duplicate" in faults:                  # SEND sent twice
        sc_list.append(dict(sc_list[0]))
    if "no_evidence" in faults:
        ev["scripted_commands"] = ev["scripted_commands"]
    return dict(tx=tx, rx=rx, evidence=ev, capture_s=capture_s, ws=ws, we=we,
                e_esp=e_esp, D=D)


class OrderedDictLike(dict):
    pass
