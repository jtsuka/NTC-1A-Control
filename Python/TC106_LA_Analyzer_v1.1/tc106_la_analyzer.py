#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tc106_la_analyzer.py  --  TC106 Formal Gate logic-analyzer (.sr) analysis tool

Implements the machine-judged items of
  B_TC106_EvidenceV3_Rerun_LA_TestSpec_v1.3 (section 4.6 / 8):
  LA-01 .. LA-09, LA-11  (LA-10 ".sr reopen in PulseView" is a manual step)
  plus V3-01 .. V3-17 / V3-BUSY from the Evidence V3 JSON (same table).

Design rules (from the spec):
  * Reads the sigrok container directly (zip + metadata + raw logic).
    It does NOT depend on PulseView decoders or their saved state.
  * Python standard library only (no numpy) -> nothing to version-pin.
  * Thresholds are constants in class Params, printed into every report.
    There are NO command-line overrides.  Change requires a new tool hash.
  * Channel meaning is NEVER hard-coded: --tx and --rx must be given by name.
  * The tool never widens a tolerance after the fact.  A missing declaration
    (e.g. offset model) yields UNRESOLVED, not PASS.

Sub-commands:
  info     container / per-channel edge statistics (+ optional decode / compare)
  analyze  Formal analysis of one run (needs .sr + Evidence V3 JSON)
  hash     print this tool's SHA256

Exit codes (analyze):  0 = all automated items PASS,  1 = some FAIL,
                       2 = no FAIL but some UNRESOLVED,  3 = usage / input error
                       (an unreadable/invalid input is never reported as 1)
"""

import argparse
import bisect
import configparser
import datetime
import hashlib
import io
import json
import os
import re
import sys
import zipfile
from collections import Counter, OrderedDict

TOOL_NAME = "tc106_la_analyzer"
TOOL_VERSION = "1.1.0"


# ----------------------------------------------------------------------------
# Parameters (spec references in comments).  Printed into every report.
# ----------------------------------------------------------------------------
class Params(object):
    # --- fixed by the spec (B v1.3) -----------------------------------------
    SLOT_US = 3300.0                # nominal slot                        (3.4)
    TX_EDGE_TOL_US = 165.0          # |T - n*3300us| <= 165us, absolute   (4.6.3)
    GAP_THRESHOLD_MS = 505.0        # 1.5 x 336.6ms rounded up            (4.6.1)
    OFFSET_SPREAD_MAX_MS = 10.0     # adopted-model spread                (4.6.4)
    BOUNDARY_MARGIN_MS = 10.0       # per-boundary DERIVED margin         (4.6.5)
                                    # (v1.1: 7 -> 10 ms by review agreement;
                                    #  covers START 4-byte reception ~4.2 ms,
                                    #  Core1/Core0 polling ~2 ms and clock drift)
    MIN_SAMPLERATE_HZ = 1000000.0   # LA-02
    POST_MARGIN_MIN_S = 10.0        # after RESULT arrival                (7)
    LOGGER_START_MAX_S = 19.0       # LA Run -> logger start              (7)
    UART_BAUD = 9600.0              # Pi -> ESP, 8N1 = 10 bit / byte      (4.6.4)
    UART_BITS_PER_BYTE = 10.0
    FORMAL_DURATION_MS = 120000     # Formal window

    # --- analysis parameters (NOT in the spec; recorded here for audit) ----
    TX_FRAME_GAP_MS = 100.0         # idle gap that separates TX frames
                                    # (worst in-frame no-edge stretch is 10
                                    #  slots = 33 ms; 3x margin)
    PI_START_TIMESTAMP_S = 0.0      # Pi time of the first EVIDENCE_START:
                                    # V3 logger takes measure_start immediately
                                    # before the first START write
    DIRB_GUARD_MIN_HIGH_US = 26000.0  # byte boundary = falling edge after HIGH
                                    # >= 26 ms (data-only HIGH <= 7 slots
                                    # = 23.1 ms, guard = 9 slots = 29.7 ms)
    DIRB_NOMINAL_BYTE_US = 17 * 3300.0
    DIRB_BYTE_WARN_FRAC = 0.18      # informational only (decoder default 18%)

    def as_dict(self):
        return OrderedDict((k, getattr(self, k)) for k in dir(self)
                           if k.isupper())


# ----------------------------------------------------------------------------
# Expected Main->TC TX frames (derived from TcMainTx slot logic; all-zero data)
#   slot0 = START (LOW), slots1..8 = 8 data bits LSB first,
#   slot9 = marker (HIGH for confirm/command byte, LOW for data byte),
#   remaining slots up to slots-per-byte = guard (HIGH).
# ----------------------------------------------------------------------------
TX_TEMPLATES = OrderedDict([
    ("SEND", dict(spb=13,
                  bytes=[(0x00, 0), (0xF3, 1), (0x00, 0), (0xFB, 1)])),
    ("RESET", dict(spb=13,
                   bytes=[(0x00, 0)] * 5 + [(0xF1, 1)] +
                         [(0x00, 0)] * 5 + [(0xF9, 1)])),
    ("SENS.ADJ", dict(spb=11, bytes=[(0xF2, 1)])),
])
TX_ORDER = ["SEND", "RESET", "SENS.ADJ"]


def template_slot_levels(tpl):
    spb = tpl["spb"]
    lv = []
    for value, marker in tpl["bytes"]:
        lv.append(0)
        for b in range(8):
            lv.append((value >> b) & 1)
        lv.append(1 if marker else 0)
        while len(lv) % spb:
            lv.append(1)
    return lv


def template_expected_edges(name):
    """[(rel_us, level_after_edge)] for the frame, idle HIGH before/after."""
    lv = template_slot_levels(TX_TEMPLATES[name])
    prev = 1
    out = []
    for s, l in enumerate(lv):
        if l != prev:
            out.append((s * Params.SLOT_US, l))
            prev = l
    return out


# ----------------------------------------------------------------------------
# sigrok .sr container
# ----------------------------------------------------------------------------
_SR_RATE = re.compile(r"^\s*([0-9]*\.?[0-9]+)\s*([kKmMgG]?)\s*(?:[hH][zZ])?\s*$")


def parse_samplerate(text):
    m = _SR_RATE.match(text)
    if not m:
        raise ValueError("cannot parse samplerate: %r" % (text,))
    mult = {"": 1.0, "k": 1e3, "K": 1e3, "m": 1e6, "M": 1e6, "g": 1e9, "G": 1e9}
    return float(m.group(1)) * mult[m.group(2)]


class SrFile(object):
    def __init__(self, path):
        self.path = path
        self.zf = zipfile.ZipFile(path, "r")
        names = self.zf.namelist()
        if "metadata" not in names:
            raise ValueError("not a sigrok session file: no 'metadata'")
        self.version = self.zf.read("version").decode("ascii", "replace").strip() \
            if "version" in names else None
        cp = configparser.ConfigParser(interpolation=None)
        cp.optionxform = str
        cp.read_string(self.zf.read("metadata").decode("utf-8", "replace"))
        self.sigrok_version = cp.get("global", "sigrok version", fallback=None)
        dev = [s for s in cp.sections() if s.startswith("device")]
        if not dev:
            raise ValueError("metadata has no [device N] section")
        d = dict(cp.items(dev[0]))
        self.samplerate_hz = parse_samplerate(d["samplerate"])
        self.unitsize = int(d.get("unitsize", "1"))
        self.total_probes = int(d.get("total probes", "8"))
        self.capturefile = d.get("capturefile", "logic-1")
        self.probes = OrderedDict()           # name -> bit index
        for k in sorted((k for k in d if re.match(r"^probe\d+$", k)),
                        key=lambda x: int(x[5:])):
            self.probes[d[k]] = int(k[5:]) - 1
        pat = re.compile(r"^%s-(\d+)$" % re.escape(self.capturefile))
        chunks = [(int(pat.match(n).group(1)), n) for n in names if pat.match(n)]
        chunks.sort()
        self.chunk_names = [n for _, n in chunks]
        if not self.chunk_names and self.capturefile in names:
            self.chunk_names = [self.capturefile]
        if not self.chunk_names:
            raise ValueError("no logic chunks found in container")
        self.n_samples = 0
        for n in self.chunk_names:
            size = self.zf.getinfo(n).file_size
            if size % self.unitsize:
                raise ValueError("chunk %s size not a multiple of unitsize" % n)
            self.n_samples += size // self.unitsize
        self.duration_s = self.n_samples / self.samplerate_hz

    def iter_chunks(self):
        for n in self.chunk_names:
            yield self.zf.read(n)

    def sha256(self):
        h = hashlib.sha256()
        with open(self.path, "rb") as f:
            for blk in iter(lambda: f.read(1 << 20), b""):
                h.update(blk)
        return h.hexdigest()


class EdgeList(object):
    """Edges of one channel.  idx = sample index of the first sample at the
    new level; lvl = new level (0/1); t_us derived from the sample rate."""

    def __init__(self, idx, lvl, initial_level, n_samples, fs):
        self.idx = idx
        self.lvl = lvl
        self.initial_level = initial_level
        self.n_samples = n_samples
        self.fs = fs
        scale = 1e6 / fs
        self.t_us = [i * scale for i in idx]

    def __len__(self):
        return len(self.idx)

    @classmethod
    def from_times_us(cls, times_us, levels, initial_level, duration_s, fs=1e6):
        """Build from ideal times (used by tests / synthetic checks)."""
        idx = [int(round(t * fs / 1e6)) for t in times_us]
        return cls(idx, list(levels), initial_level, int(round(duration_s * fs)), fs)


def invert_edgelist(el):
    """Logical inversion (used when TX is probed at the Q2 gate, where the
    polarity is opposite to the bus).  Recorded in the report."""
    return EdgeList(list(el.idx), [1 - l for l in el.lvl], 1 - el.initial_level,
                    el.n_samples, el.fs)


def _bit_table(bit):
    return bytes(((v >> bit) & 1) for v in range(256))


def extract_edges(sr, channel_names, progress=None):
    """One pass over the container; returns {name: EdgeList}.
    Uses bytes.translate + bytes.find (C speed); Python-level work is
    proportional to the number of edges only."""
    chans = OrderedDict()
    for name in channel_names:
        if name not in sr.probes:
            raise ValueError("channel %r not in metadata probes %s"
                             % (name, list(sr.probes)))
        bit = sr.probes[name]
        if bit // 8 >= sr.unitsize:
            raise ValueError("channel %r bit %d outside unitsize %d"
                             % (name, bit, sr.unitsize))
        chans[name] = dict(lane=bit // 8, table=_bit_table(bit % 8),
                           idx=[], lvl=[], cur=None)
    base = 0
    u = sr.unitsize
    n_chunks = len(sr.chunk_names)
    for ci, chunk in enumerate(sr.iter_chunks()):
        n = len(chunk) // u
        for c in chans.values():
            lane = chunk if u == 1 else chunk[c["lane"]::u]
            bits = lane.translate(c["table"])
            cur = c["cur"]
            if cur is None:
                cur = bits[0]
                c["initial"] = cur
            pos = 0
            idx, lvl = c["idx"], c["lvl"]
            find = bits.find
            while True:
                nxt = find(b"\x01" if cur == 0 else b"\x00", pos)
                if nxt < 0:
                    break
                cur ^= 1
                idx.append(base + nxt)
                lvl.append(cur)
                pos = nxt + 1
            c["cur"] = cur
        base += n
        if progress:
            progress(ci + 1, n_chunks)
    out = OrderedDict()
    for name, c in chans.items():
        out[name] = EdgeList(c["idx"], c["lvl"], c.get("initial", 1),
                             sr.n_samples, sr.samplerate_hz)
    return out


# ----------------------------------------------------------------------------
# helpers
# ----------------------------------------------------------------------------
def _spread(vals):
    return (max(vals) - min(vals)) if vals else None


def _fmt(x, nd=3):
    return None if x is None else round(x, nd)


def level_at(t_us_list, lvl_list, initial, t):
    i = bisect.bisect_right(t_us_list, t) - 1
    return lvl_list[i] if i >= 0 else initial


# ----------------------------------------------------------------------------
# Main->TC TX analysis
# ----------------------------------------------------------------------------
def segment_frames(t_us, lvl, gap_us):
    frames = []
    start = 0
    for i in range(1, len(t_us)):
        if t_us[i] - t_us[i - 1] > gap_us:
            frames.append((start, i))
            start = i
    if t_us:
        frames.append((start, len(t_us)))
    return frames


def _decode_tx(lev, s_last, spb):
    out, errs, k = [], [], 0
    while k * spb <= s_last:
        base = k * spb
        if lev[base] != 0:
            errs.append("byte%d:START_not_LOW" % k)
            break
        val = 0
        for b in range(8):
            val |= lev[base + 1 + b] << b
        marker = lev[base + 9]
        for g in range(10, spb):
            if base + g <= s_last and lev[base + g] != 1:
                errs.append("byte%d:guard_slot%d_not_HIGH" % (k, g))
        out.append((val, marker))
        k += 1
    return out, errs


def analyze_tx_frame(times, levels, P):
    slot = P.SLOT_US
    t0 = times[0]
    n = len(times)
    res = OrderedDict(start_us=t0, n_edges=n, errors=[], name=None)
    if levels[0] != 0:
        res["errors"].append("first_edge_not_falling")
    slots = [int(round((t - t0) / slot)) for t in times]
    for i in range(1, n):
        if slots[i] == slots[i - 1]:
            res["errors"].append("edges_share_slot@%d" % i)
    s_last = slots[-1]
    lev = [1] * (s_last + 1 + 16)
    cur, e = 1, 0
    for s in range(len(lev)):
        while e < n and slots[e] <= s:
            cur = levels[e]
            e += 1
        lev[s] = cur

    matched = None
    for name in TX_ORDER:
        tpl = TX_TEMPLATES[name]
        b, errs = _decode_tx(lev, s_last, tpl["spb"])
        if not errs and b == tpl["bytes"]:
            matched = name
            break
    # diagnostics decode (both strides) for unmatched frames
    diag = OrderedDict()
    for spb in (13, 11):
        b, errs = _decode_tx(lev, s_last, spb)
        diag["spb%d" % spb] = OrderedDict(
            bytes=" ".join("%02X%s" % (v, "*" if m else "") for v, m in b),
            errors=errs)
    res["decode_diag"] = diag
    res["name"] = matched

    # --- edge-interval rule (LA-11): |T - n*3300| <= 165us, absolute -------
    ivals = []
    exp = template_expected_edges(matched) if matched else None
    edge_count_ok = None
    if exp is not None:
        edge_count_ok = (len(exp) == n)
        if not edge_count_ok:
            res["errors"].append("edge_count %d != expected %d" % (n, len(exp)))
    max_dev = 0.0
    tol_fail = 0
    n_mismatch = 0
    for i in range(n - 1):
        T = times[i + 1] - times[i]
        n_obs = int(round(T / slot))
        dev = T - n_obs * slot
        n_exp = None
        if exp is not None and edge_count_ok:
            n_exp = int(round((exp[i + 1][0] - exp[i][0]) / slot))
            if n_exp != n_obs:
                n_mismatch += 1
        ok = (abs(dev) <= P.TX_EDGE_TOL_US) and (n_exp is None or n_exp == n_obs)
        if abs(dev) > P.TX_EDGE_TOL_US:
            tol_fail += 1
        max_dev = max(max_dev, abs(dev))
        ivals.append(OrderedDict(i=i, T_us=_fmt(T, 1), n_obs=n_obs, n_exp=n_exp,
                                 dev_us=_fmt(dev, 1), ok=ok))
    res["interval_rule"] = OrderedDict(
        checked=len(ivals), tol_us=P.TX_EDGE_TOL_US,
        max_abs_dev_us=_fmt(max_dev, 1), n_outside_tol=tol_fail,
        n_slotcount_mismatch=n_mismatch,
        bad=[iv for iv in ivals if not iv["ok"]][:20])

    # --- byte-start spacing (13-step / 11-step structure) -------------------
    # Gating rule (spec 4.6.2 item 3): ADJACENT byte-start interval must satisfy
    # the 4.6.3 rule |T - spb*3300us| <= 165us.  The cumulative deviation from
    # the ideal grid is reported for information only: over a RESET frame
    # (~0.5 s) the LA's own clock tolerance (order of 100-200 ppm) alone can
    # contribute ~100 us, so it must not gate.
    if matched and edge_count_ok:
        spb = TX_TEMPLATES[matched]["spb"]
        starts = []
        for j, (rel, l) in enumerate(exp):
            if l == 0 and abs((rel / slot) % spb) < 1e-9:
                starts.append((j, rel))
        st_t = [times[j] for j, _ in starts]
        adj = [(st_t[i + 1] - st_t[i]) - spb * slot for i in range(len(st_t) - 1)]
        cum = [(times[j] - t0) - rel for j, rel in starts]
        res["byte_start"] = OrderedDict(
            expected_spacing_us=spb * slot, n_bytes=len(starts),
            adjacent_max_abs_dev_us=_fmt(max(abs(d) for d in adj), 1) if adj else None,
            cumulative_max_abs_dev_us=_fmt(max(abs(d) for d in cum), 1) if cum else None,
            ok=all(abs(d) <= P.TX_EDGE_TOL_US for d in adj))
    res["decoded_bytes"] = (" ".join("%02X" % v for v, _ in TX_TEMPLATES[matched]["bytes"])
                            if matched else None)
    return res


def analyze_tx(tx, P):
    gap = P.TX_FRAME_GAP_MS * 1000.0
    out = OrderedDict(frames=[], initial_level=tx.initial_level)
    for a, b in segment_frames(tx.t_us, tx.lvl, gap):
        fr = analyze_tx_frame(tx.t_us[a:b], tx.lvl[a:b], P)
        out["frames"].append(fr)
    return out


# ----------------------------------------------------------------------------
# Direction-B (TC->Main) decode from edges  (independent of ESP and PulseView)
#   byte = falling edge after HIGH >= 26 ms; 7 data bits LSB first sampled at
#   slot centres; frame = 5 data bytes + footer 0x7F.
# ----------------------------------------------------------------------------
def decode_dirb(t_us, lvl, initial, P):
    slot = P.SLOT_US
    bounds = []
    for j in range(1, len(t_us)):
        if lvl[j] == 0 and lvl[j - 1] == 1 and \
                (t_us[j] - t_us[j - 1]) >= P.DIRB_GUARD_MIN_HIGH_US:
            bounds.append(t_us[j])
    frames, warn_int, sync_miss, bytes_dec = [], 0, 0, 0
    aligned = False
    data, first_t = [], None
    intervals = []
    for k, b in enumerate(bounds):
        val = 0
        for bit in range(7):
            tt = b + (bit + 1) * slot + slot / 2.0
            val |= level_at(t_us, lvl, initial, tt) << bit
        bytes_dec += 1
        if k + 1 < len(bounds):
            iv = bounds[k + 1] - b
            intervals.append(iv)
            if abs(iv - P.DIRB_NOMINAL_BYTE_US) > P.DIRB_BYTE_WARN_FRAC * P.DIRB_NOMINAL_BYTE_US:
                warn_int += 1
        if val == 0x7F:
            if aligned and len(data) == 5:
                frames.append(OrderedDict(t_start_us=first_t, t_footer_us=b,
                                          payload=list(data)))
            elif aligned:
                sync_miss += 1
            aligned = True
            data, first_t = [], None
        else:
            if not data:
                first_t = b
            data.append(val)
            if len(data) > 5:
                pass  # over-long span: wait for the footer (counted at footer)
    return OrderedDict(bounds=bounds, frames=frames, bytes_decoded=bytes_dec,
                       sync_misses=sync_miss, interval_warn=warn_int,
                       byte_interval_min_us=min(intervals) if intervals else None,
                       byte_interval_max_us=max(intervals) if intervals else None)


# ----------------------------------------------------------------------------
# Evidence V3 JSON items (V3-01 .. V3-17, V3-BUSY)
# ----------------------------------------------------------------------------
# Pi -> ESP packets exactly as sent by pi_logger_evidence_v3.py (all data 0)
EXPECTED_PI_PACKETS = OrderedDict([
    ("SEND", "02 00 00 00 00 02"),
    ("RESET", "01 00 00 00 00 00 00 00 00 00 00 01"),
    ("SENS.ADJ", "03 00 00 00 00 00 00 03"),
])


def _norm_hex(text):
    return " ".join((text or "").upper().split())


def _checksum7_matches(hex_text):
    try:
        b = bytes.fromhex(hex_text)
    except ValueError:
        return False
    return len(b) >= 2 and (sum(b[:-1]) & 0x7F) == b[-1]


V3_DELTA_RULES = [
    ("V3-02", "tx_send_started_count", 1),
    ("V3-03", "tx_reset_started_count", 1),
    ("V3-04", "tx_sens_started_count", 1),
    ("V3-05", "tx_done_count", 3),
    ("V3-06", "tx_aborted_done_count", 0),
    ("V3-07", "tc_main_tx_overrun_count", 0),
    ("V3-08", "c1_edge_overflow", 0),
    ("V3-09", "c2_decode_errors", 0),
    ("V3-10", "c2_timing_errors", 0),
    ("V3-11", "c2_local_edge_overflow", 0),
    ("V3-12", "c3_frame_errors", 0),
    ("V3-13", "c3_sync_misses", 0),
    ("V3-14", "c3_resyncs", 0),
    ("V3-15", "c4_queue_drops", 0),
]


def evaluate_v3(ev, P, formal):
    R = OrderedDict()

    def add(i, status, detail, **kw):
        R[i] = OrderedDict(status=status, detail=detail, **kw)

    if ev is None:
        return R
    got = ev.get("evidence_received", False)
    if not got or "counters" not in ev:
        for i in ["V3-01"] + [r[0] for r in V3_DELTA_RULES]:
            add(i, "UNRESOLVED", "Evidence RESULT not received in this JSON")
    else:
        hdr = [ev.get(k) for k in ("magic_ok", "version_ok", "length_ok",
                                   "checksum_ok", "duration_ok")]
        ok = all(h is True for h in hdr) and ev.get("version") == 2
        if formal and ev.get("duration_ms") != P.FORMAL_DURATION_MS:
            ok = False
        add("V3-01", "PASS" if ok else "FAIL",
            "magic/version/length/checksum/duration flags=%s version=%s duration_ms=%s"
            % (hdr, ev.get("version"), ev.get("duration_ms")))
        c = ev["counters"]
        for i, name, want in V3_DELTA_RULES:
            if name not in c:
                add(i, "UNRESOLVED", "counter %s missing" % name)
                continue
            d = c[name].get("delta")
            add(i, "PASS" if d == want else "FAIL",
                "%s delta=%s (required %s)" % (name, d, want), value=d)
    # V3-CMD (tool-defined, proposed for the B section-8 table): the three Pi
    # packets are exactly the specified all-zero packets and carry a valid
    # checksum7.  Recomputed here; the logger's own checksum7_ok flag must agree.
    sc = ev.get("scripted_commands")
    if not sc:
        add("V3-CMD", "UNRESOLVED", "no scripted_commands in the JSON")
    else:
        issues = []
        seen = OrderedDict()
        for c in sc:
            seen.setdefault(c.get("name"), []).append(c)
        for n, want in EXPECTED_PI_PACKETS.items():
            lst = seen.get(n, [])
            if len(lst) != 1:
                issues.append("%s sent %d time(s) (required 1)" % (n, len(lst)))
                continue
            rh = _norm_hex(lst[0].get("raw_hex"))
            if rh != want:
                issues.append("%s raw_hex '%s' != expected '%s'" % (n, rh, want))
            if not _checksum7_matches(rh):
                issues.append("%s checksum7 recomputation failed" % n)
            if lst[0].get("checksum7_ok") is not True:
                issues.append("%s logger checksum7_ok=%r" % (n, lst[0].get("checksum7_ok")))
        for n in seen:
            if n not in EXPECTED_PI_PACKETS:
                issues.append("unexpected command %r sent" % (n,))
        add("V3-CMD", "PASS" if not issues else "FAIL",
            "SEND/RESET/SENS.ADJ packets exact + checksum7 valid" if not issues
            else "; ".join(issues))
    nt = ev.get("normal_telemetry")
    if nt:
        bad = nt.get("frames_received", 0) - nt.get("footer_ok_count", 0)
        add("V3-16", "PASS" if bad == 0 else "FAIL",
            "frames_received=%s footer_ok_count=%s (footer bad=%s)"
            % (nt.get("frames_received"), nt.get("footer_ok_count"), bad))
        mi = nt.get("max_interval_ms")
        add("V3-17", "PASS" if (mi is not None and mi <= P.GAP_THRESHOLD_MS) else "FAIL",
            "Pi max_interval_ms=%s (threshold %.0f)" % (mi, P.GAP_THRESHOLD_MS),
            value=mi)
    else:
        add("V3-16", "UNRESOLVED", "no normal_telemetry")
        add("V3-17", "UNRESOLVED", "no normal_telemetry")
    c = (ev.get("counters") or {})
    if "poll_busy_over1000_count_window" in c:
        d = c["poll_busy_over1000_count_window"]["delta"]
        add("V3-BUSY", "AUX_PASS" if d == 0 else "AUX_NONZERO",
            "poll_busy_over1000_count_window delta=%s (auxiliary, not gating)" % d,
            value=d)
    return R


# ----------------------------------------------------------------------------
# Formal analysis
# ----------------------------------------------------------------------------
def _pi_commands(ev):
    if not ev:
        return None
    cmds = OrderedDict()
    for c in ev.get("scripted_commands", []) or []:
        cmds.setdefault(c["name"], []).append(c)
    return cmds


def analyze_edges(cap, tx, rx, ev, model, P, formal):
    """cap: dict(fs, duration_s).  tx/rx: EdgeList.  ev: Evidence V3 JSON dict
    or None.  model: None | 'RAW' | 'WIRE_CORRECTED'."""
    R = OrderedDict()
    notes = []

    def add(i, status, detail, **kw):
        R[i] = OrderedDict(status=status, detail=detail, **kw)

    txa = analyze_tx(tx, P)
    frames = txa["frames"]
    by_name = OrderedDict((n, [f for f in frames if f["name"] == n]) for n in TX_ORDER)
    unknown = [f for f in frames if f["name"] is None]

    # --- LA-02 / LA-03 ------------------------------------------------------
    add("LA-02", "PASS" if cap["fs"] >= P.MIN_SAMPLERATE_HZ else "FAIL",
        "samplerate %.0f Hz (>= %.0f required)" % (cap["fs"], P.MIN_SAMPLERATE_HZ))
    add("LA-03", "PASS" if (len(tx) > 0 and len(rx) > 0) else "FAIL",
        "TX edges=%d, RX(TP5) edges=%d in the same .sr" % (len(tx), len(rx)))

    # --- LA-04: frame counts -----------------------------------------------
    counts = OrderedDict((n, len(v)) for n, v in by_name.items())
    ok4 = all(counts[n] == 1 for n in TX_ORDER) and not unknown
    add("LA-04", "PASS" if ok4 else "FAIL",
        "SEND=%d RESET=%d SENS.ADJ=%d unknown/extra events=%d (required 1/1/1/0)"
        % (counts["SEND"], counts["RESET"], counts["SENS.ADJ"], len(unknown)),
        frames_at_us=[_fmt(f["start_us"], 1) for f in frames],
        unknown=[OrderedDict(start_us=_fmt(f["start_us"], 1), n_edges=f["n_edges"],
                             decode_diag=f["decode_diag"]) for f in unknown])

    # --- LA-05: completeness -------------------------------------------------
    complete = [f for f in frames if f["name"] and
                not any("edge_count" in e or "edges_share" in e for e in f["errors"])]
    ok5 = (len(complete) == len(frames) == 3)
    add("LA-05", "PASS" if ok5 else "FAIL",
        "%d of %d TX events decode to a complete expected frame with the expected "
        "edge count" % (len(complete), len(frames)))

    # --- LA-06: structure ----------------------------------------------------
    parts, ok6 = [], (len(frames) == 3 and not unknown)
    for f in frames:
        if not f["name"]:
            ok6 = False
            parts.append("unmatched frame (%d edges)" % f["n_edges"])
            continue
        multi = len(TX_TEMPLATES[f["name"]]["bytes"]) > 1
        bs = f.get("byte_start")
        good = (bool(bs and bs["ok"]) if multi else True)
        if not good:
            ok6 = False
        if multi:
            parts.append("%s: bytes %s, adjacent byte-start spacing %.1f ms, max dev "
                         "%s us (<= %.0f) %s" % (
                             f["name"], f["decoded_bytes"],
                             (bs["expected_spacing_us"] / 1000.0) if bs else 0.0,
                             bs["adjacent_max_abs_dev_us"] if bs else "n/a",
                             P.TX_EDGE_TOL_US, "OK" if good else "OUT OF TOLERANCE"))
        else:
            parts.append("%s: byte %s" % (f["name"], f["decoded_bytes"]))
    add("LA-06", "PASS" if ok6 else "FAIL", "; ".join(parts),
        not_observable=["SENS.ADJ 11-step guard (single byte; trailing guard "
                        "merges with idle HIGH) - only the byte value F2 and "
                        "its marker are verified",
                        "trailing guard slots of the last byte of SEND/RESET"])

    # --- LA-11: TX edge timing ----------------------------------------------
    any_out = sum(f["interval_rule"]["n_outside_tol"] for f in frames)
    any_n = sum(f["interval_rule"]["n_slotcount_mismatch"] for f in frames)
    max_dev = max([f["interval_rule"]["max_abs_dev_us"] or 0 for f in frames] or [0])
    n_int = sum(f["interval_rule"]["checked"] for f in frames)
    if not frames:
        add("LA-11", "FAIL", "no TX frames")
    elif any_out or any_n:
        add("LA-11", "FAIL", "%d interval(s) outside +-%.0f us, %d slot-count "
            "mismatch(es); max |dev|=%.1f us over %d intervals"
            % (any_out, P.TX_EDGE_TOL_US, any_n, max_dev, n_int),
            bad=[dict(frame=f["name"], **iv) for f in frames
                 for iv in f["interval_rule"]["bad"]][:20])
    elif unknown:
        add("LA-11", "UNRESOLVED", "intervals within tolerance (max |dev|=%.1f us) "
            "but %d frame(s) not identified, slot counts unverifiable"
            % (max_dev, len(unknown)))
    else:
        add("LA-11", "PASS", "%d intervals, all |T-n*3300us| <= %.0f us "
            "(max %.1f us), slot counts match expected"
            % (n_int, P.TX_EDGE_TOL_US, max_dev))

    # --- LA/Pi correspondence and offsets (LA-08) ---------------------------
    cmds = _pi_commands(ev)
    offsets = None
    unresolved_reason = None
    if cmds is None:
        unresolved_reason = "no Evidence JSON"
    elif not all(len(cmds.get(n, [])) == 1 for n in TX_ORDER):
        unresolved_reason = "Pi JSON does not contain exactly one of each command"
    elif not ok4:
        unresolved_reason = "LA-04 failed: TX events not uniquely identified"
    if unresolved_reason is None:
        la = OrderedDict((n, by_name[n][0]["start_us"] / 1e6) for n in TX_ORDER)
        pi = OrderedDict((n, cmds[n][0]["sent_monotonic_s"]) for n in TX_ORDER)
        wire = OrderedDict()
        for n in TX_ORDER:
            rh = cmds[n][0].get("raw_hex") or ""
            nbytes = len(rh.split()) or {"SEND": 6, "RESET": 12, "SENS.ADJ": 8}[n]
            wire[n] = nbytes * P.UART_BITS_PER_BYTE / P.UART_BAUD
        raw = OrderedDict((n, la[n] - pi[n]) for n in TX_ORDER)
        wc = OrderedDict((n, raw[n] - wire[n]) for n in TX_ORDER)
        order_ok = (la["SEND"] < la["RESET"] < la["SENS.ADJ"] and
                    pi["SEND"] < pi["RESET"] < pi["SENS.ADJ"])
        offsets = dict(la_s=la, pi_s=pi, wire_s=wire, RAW=raw, WIRE_CORRECTED=wc,
                       order_ok=order_ok,
                       spread_ms=OrderedDict(
                           RAW=_spread(list(raw.values())) * 1e3,
                           WIRE_CORRECTED=_spread(list(wc.values())) * 1e3))
    if offsets is None:
        add("LA-08", "UNRESOLVED", "cannot correlate LA and Pi: " + unresolved_reason)
    else:
        sp = offsets["spread_ms"]
        info = "raw spread %.3f ms, wire-corrected spread %.3f ms; order %s" % (
            sp["RAW"], sp["WIRE_CORRECTED"], "unique/OK" if offsets["order_ok"] else "MISMATCH")
        if model is None:
            add("LA-08", "UNRESOLVED", "offset model not declared (" + info + ")",
                offsets_ms=OrderedDict(
                    (m, OrderedDict((n, _fmt(offsets[m][n] * 1e3)) for n in TX_ORDER))
                    for m in ("RAW", "WIRE_CORRECTED")))
        else:
            ok8 = offsets["order_ok"] and sp[model] <= P.OFFSET_SPREAD_MAX_MS
            add("LA-08", "PASS" if ok8 else "FAIL",
                "declared model %s spread %.3f ms (<= %.1f required); %s"
                % (model, sp[model], P.OFFSET_SPREAD_MAX_MS, info),
                offsets_ms=OrderedDict(
                    (m, OrderedDict((n, _fmt(offsets[m][n] * 1e3)) for n in TX_ORDER))
                    for m in ("RAW", "WIRE_CORRECTED")))

    # --- windows --------------------------------------------------------------
    dur_ms = ev.get("duration_ms") if ev else None
    window = None
    sens = OrderedDict()
    if offsets is not None and dur_ms:
        def win_for(m):
            offs = list(offsets[m].values())
            s0 = P.PI_START_TIMESTAMP_S
            mg = P.BOUNDARY_MARGIN_MS / 1e3
            lo, hi = s0 + min(offs) - mg, s0 + max(offs) + mg
            d = dur_ms / 1e3
            return OrderedDict(start_lo_s=lo, start_hi_s=hi,
                               end_lo_s=lo + d, end_hi_s=hi + d,
                               mean_offset_s=sum(offs) / len(offs))
        for m in ("RAW", "WIRE_CORRECTED"):
            sens[m] = win_for(m)
        if model is not None:
            window = sens[model]

    def edges_in(tl, lo_s, hi_s, closed_lo=True, closed_hi=True):
        lo, hi = lo_s * 1e6, hi_s * 1e6
        a = bisect.bisect_left(tl, lo) if closed_lo else bisect.bisect_right(tl, lo)
        b = bisect.bisect_right(tl, hi) if closed_hi else bisect.bisect_left(tl, hi)
        return max(0, b - a)

    # --- LA-01: capture range ------------------------------------------------
    if window is None:
        add("LA-01", "UNRESOLVED",
            "window cannot be placed on the LA time axis (needs identified TX "
            "frames, Pi JSON, duration_ms and a declared offset model)")
    else:
        cap_s = cap["duration_s"]
        inside = window["start_lo_s"] >= 0.0 and window["end_hi_s"] <= cap_s
        D = P.PI_START_TIMESTAMP_S + window["mean_offset_s"]
        if ev.get("evidence_received") and ev.get("evidence_monotonic_s") is not None:
            res_la = ev["evidence_monotonic_s"] + window["mean_offset_s"]
            post = cap_s - res_la
            post_note = "post-RESULT margin %.2f s" % post
        else:
            post = cap_s - window["end_hi_s"]
            post_note = ("RESULT time unknown; post margin from window end %.2f s "
                         "(conservative proxy)" % post)
        ok1 = inside and post >= P.POST_MARGIN_MIN_S and 0.0 <= D <= P.LOGGER_START_MAX_S
        add("LA-01", "PASS" if ok1 else "FAIL",
            "capture %.1f s; window [%.3f, %.3f] s %s; logger start delay D=%.2f s "
            "(<= %.0f); %s (>= %.0f)"
            % (cap_s, window["start_lo_s"], window["end_hi_s"],
               "inside" if inside else "NOT inside", D, P.LOGGER_START_MAX_S,
               post_note, P.POST_MARGIN_MIN_S),
            pre_margin_s=_fmt(window["start_lo_s"], 3), post_margin_s=_fmt(post, 3))

    # --- TP5 decode, LA-07 -----------------------------------------------------
    dec = decode_dirb(rx.t_us, rx.lvl, rx.initial_level, P)
    if window is None:
        add("LA-07", "UNRESOLVED", "window not placed (see LA-01)",
            frames_whole_capture=len(dec["frames"]))
    else:
        lo, hi = window["start_hi_s"] * 1e6, window["end_lo_s"] * 1e6
        fr = [f for f in dec["frames"] if lo <= f["t_start_us"] <= hi]
        marks = [lo] + [f["t_start_us"] for f in fr] + [hi]
        gaps = [(marks[i + 1] - marks[i]) / 1e3 for i in range(len(marks) - 1)]
        max_gap = max(gaps) if gaps else None
        ia = bisect.bisect_left(rx.t_us, lo)
        ib = bisect.bisect_right(rx.t_us, hi)
        seq = [lo] + rx.t_us[ia:ib] + [hi]
        egap = max((seq[i + 1] - seq[i]) for i in range(len(seq) - 1)) / 1e3
        ok7 = bool(fr) and max_gap <= P.GAP_THRESHOLD_MS and egap <= P.GAP_THRESHOLD_MS
        add("LA-07", "PASS" if ok7 else "FAIL",
            "%d Direction-B frames decoded in the core window; max frame gap %.1f ms, "
            "max edge gap %.1f ms (threshold %.0f ms); sync misses (whole capture) %d"
            % (len(fr), max_gap if max_gap is not None else -1, egap,
               P.GAP_THRESHOLD_MS, dec["sync_misses"]),
            payloads=sorted(set(" ".join("%02d" % b for b in f["payload"]) for f in fr))[:10],
            byte_interval_warnings=dec["interval_warn"])

    # --- LA-09: edge count vs c1_isr_edges ------------------------------------
    def la09_for(w):
        Es = edges_in(rx.t_us, w["start_lo_s"], w["start_hi_s"])
        Ee = edges_in(rx.t_us, w["end_lo_s"], w["end_hi_s"])
        Ec = edges_in(rx.t_us, w["start_hi_s"], w["end_lo_s"], False, False)
        return Ec, Es, Ee

    counters = (ev or {}).get("counters") or {}
    if window is None or "c1_isr_edges" not in counters:
        add("LA-09", "UNRESOLVED",
            "needs placed window and c1_isr_edges in the Evidence JSON")
    else:
        E_ESP = counters["c1_isr_edges"]["delta"]
        Ec, Es, Ee = la09_for(window)
        lo_b, hi_b = Ec, Ec + Es + Ee
        ok9 = lo_b <= E_ESP <= hi_b
        sens_la09 = OrderedDict()
        for m, w in sens.items():
            c2, s2, e2 = la09_for(w)
            sens_la09[m] = OrderedDict(E_LA_min=c2, E_LA_max=c2 + s2 + e2,
                                       within=(c2 <= E_ESP <= c2 + s2 + e2))
        add("LA-09", "PASS" if ok9 else "FAIL",
            "E_ESP(c1_isr_edges delta)=%d; LA core=%d, start-band=%d, end-band=%d "
            "-> allowed [%d, %d]" % (E_ESP, Ec, Es, Ee, lo_b, hi_b),
            E_ESP=E_ESP, E_LA_core=Ec, E_start=Es, E_end=Ee,
            E_LA_min=lo_b, E_LA_max=hi_b, sensitivity_other_model=sens_la09)

    add("LA-10", "MANUAL", "reopen the saved .sr in PulseView and confirm the six "
        "points of section 4.1A-D (not machine-checkable)")

    V = evaluate_v3(ev, P, formal)
    if formal and ev is not None and ev.get("duration_ms") != P.FORMAL_DURATION_MS:
        notes.append("FORMAL: duration_ms=%s != %d" % (ev.get("duration_ms"),
                                                     P.FORMAL_DURATION_MS))
    gating = [k for k in list(V) + list(R) if k not in ("LA-10", "V3-BUSY")]
    allres = OrderedDict(list(V.items()) + list(R.items()))
    stat = [allres[k]["status"] for k in gating]
    if any(s == "FAIL" for s in stat):
        overall = "HAS_FAIL"
    elif any(s == "UNRESOLVED" for s in stat) or notes:
        overall = "HAS_UNRESOLVED"
    else:
        overall = "ALL_AUTOMATED_ITEMS_PASS"
    return OrderedDict(overall=overall, notes=notes, results_v3=V, results_la=R,
                       tx=txa, window_la_s=window,
                       window_sensitivity=sens or None,
                       dirb=OrderedDict(
                           frames_whole_capture=len(dec["frames"]),
                           bytes_decoded=dec["bytes_decoded"],
                           sync_misses=dec["sync_misses"],
                           byte_interval_min_ms=_fmt((dec["byte_interval_min_us"] or 0) / 1e3),
                           byte_interval_max_ms=_fmt((dec["byte_interval_max_us"] or 0) / 1e3)))


# ----------------------------------------------------------------------------
# CLI
# ----------------------------------------------------------------------------
def tool_sha256():
    with open(os.path.abspath(__file__), "rb") as f:
        return hashlib.sha256(f.read()).hexdigest()


def _progress(i, n):
    sys.stderr.write("\r  reading chunk %d/%d" % (i, n))
    sys.stderr.flush()
    if i == n:
        sys.stderr.write("\n")


def channel_stats(el):
    if len(el) == 0:
        return OrderedDict(edges=0, initial_level=el.initial_level)
    iv = [el.idx[i + 1] - el.idx[i] for i in range(len(el) - 1)]
    scale = 1e6 / el.fs
    return OrderedDict(
        edges=len(el), rising=sum(1 for l in el.lvl if l == 1),
        falling=sum(1 for l in el.lvl if l == 0), initial_level=el.initial_level,
        first_edge_ms=_fmt(el.idx[0] * scale / 1e3, 3),
        last_edge_s=_fmt(el.idx[-1] * scale / 1e6, 6),
        max_edge_interval_ms=_fmt(max(iv) * scale / 1e3, 3) if iv else None,
        min_edge_interval_ms=_fmt(min(iv) * scale / 1e3, 3) if iv else None)


def cmd_info(a):
    P = Params()
    sr = SrFile(a.sr)
    out = OrderedDict(tool=OrderedDict(name=TOOL_NAME, version=TOOL_VERSION,
                                       sha256=tool_sha256()),
                      input=OrderedDict(path=os.path.basename(a.sr), sha256=sr.sha256(),
                                        size_bytes=os.path.getsize(a.sr)),
                      container=OrderedDict(
                          sigrok_version=sr.sigrok_version, version_file=sr.version,
                          samplerate_hz=sr.samplerate_hz, unitsize=sr.unitsize,
                          total_probes=sr.total_probes, probes=sr.probes,
                          chunks=len(sr.chunk_names), raw_samples=sr.n_samples,
                          duration_s=_fmt(sr.duration_s, 6)))
    names = list(sr.probes)
    edges = extract_edges(sr, names, _progress if a.progress else None)
    out["channels"] = OrderedDict((n, channel_stats(e)) for n, e in edges.items())
    if a.compare:
        x, y = a.compare
        ex, ey = edges[x], edges[y]
        cmpd = OrderedDict(a=x, b=y, edges_a=len(ex), edges_b=len(ey))
        if len(ex) == len(ey):
            h = Counter(ey.idx[i] - ex.idx[i] for i in range(len(ex)))
            cmpd["delta_samples_b_minus_a"] = OrderedDict(
                (str(k), v) for k, v in sorted(h.items()))
            cmpd["max_abs_delta_samples"] = max(abs(k) for k in h) if h else 0
        out["compare"] = cmpd
    for ch in (a.dirb or []):
        e = edges[ch]
        d = decode_dirb(e.t_us, e.lvl, e.initial_level, P)
        pl = Counter(" ".join("%02X" % b for b in f["payload"]) for f in d["frames"])
        iv = [(d["frames"][i + 1]["t_start_us"] - d["frames"][i]["t_start_us"]) / 1e3
              for i in range(len(d["frames"]) - 1)]
        out.setdefault("direction_b_decode", OrderedDict())[ch] = OrderedDict(
            frames=len(d["frames"]), payloads=OrderedDict(pl.most_common(8)),
            sync_misses=d["sync_misses"], bytes_decoded=d["bytes_decoded"],
            byte_interval_ms_min_max=[_fmt((d["byte_interval_min_us"] or 0) / 1e3),
                                      _fmt((d["byte_interval_max_us"] or 0) / 1e3)],
            byte_interval_warnings=d["interval_warn"],
            frame_interval_ms_min_max=[_fmt(min(iv)), _fmt(max(iv))] if iv else None)
    txt = json.dumps(out, ensure_ascii=False, indent=2)
    print(txt)
    if a.out_json:
        _write_new(a.out_json, txt + "\n")
    return 0


def _write_new(path, text):
    if os.path.exists(path):
        sys.stderr.write("ABORT: refusing to overwrite existing file %s\n" % path)
        sys.exit(3)
    with io.open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write(text)


def cmd_analyze(a):
    P = Params()
    if a.formal and (not a.tx or not a.rx or not a.offset_model):
        sys.stderr.write("--formal requires --tx, --rx and --offset-model\n")
        return 3
    sr = SrFile(a.sr)
    with io.open(a.evidence_json, "r", encoding="utf-8") as f:
        ev = json.load(f)
    with open(a.evidence_json, "rb") as f:
        ev_sha = hashlib.sha256(f.read()).hexdigest()
    edges = extract_edges(sr, [a.tx, a.rx], _progress if a.progress else None)
    cap = dict(fs=sr.samplerate_hz, duration_s=sr.duration_s)
    tx_el = invert_edgelist(edges[a.tx]) if a.tx_invert else edges[a.tx]
    res = analyze_edges(cap, tx_el, edges[a.rx], ev, a.offset_model, P, a.formal)
    report = OrderedDict(
        tool=OrderedDict(name=TOOL_NAME, version=TOOL_VERSION, sha256=tool_sha256()),
        generated_at=datetime.datetime.now().astimezone().isoformat(timespec="seconds"),
        mode="FORMAL" if a.formal else "EXPLORATORY",
        inputs=OrderedDict(
            sr=OrderedDict(path=os.path.basename(a.sr), sha256=sr.sha256(),
                           samplerate_hz=sr.samplerate_hz, samples=sr.n_samples,
                           duration_s=_fmt(sr.duration_s, 6),
                           sigrok_version=sr.sigrok_version, probes=sr.probes),
            evidence_json=OrderedDict(path=os.path.basename(a.evidence_json),
                                      sha256=ev_sha),
            channels=OrderedDict(tx=a.tx, rx_tp5=a.rx, tx_inverted=bool(a.tx_invert)),
            offset_model_declared=a.offset_model),
        parameters=P.as_dict(),
        channel_stats=OrderedDict((n, channel_stats(edges[n])) for n in (a.tx, a.rx)),
        **res)
    txt = json.dumps(report, ensure_ascii=False, indent=2)
    if a.out_json:
        _write_new(a.out_json, txt + "\n")
    _print_summary(report)
    if a.out_json:
        print("\nreport written: %s" % a.out_json)
    return {"HAS_FAIL": 1, "HAS_UNRESOLVED": 2}.get(res["overall"], 0)


def _print_summary(rep):
    print("=" * 78)
    print("%s v%s  sha256=%s" % (rep["tool"]["name"], rep["tool"]["version"],
                                 rep["tool"]["sha256"][:16] + "..."))
    print("mode=%s  .sr sha256=%s" % (rep["mode"], rep["inputs"]["sr"]["sha256"][:16] + "..."))
    print("channels: TX=%s%s  TP5=%s   offset model: %s"
          % (rep["inputs"]["channels"]["tx"],
             " (inverted)" if rep["inputs"]["channels"].get("tx_inverted") else "",
             rep["inputs"]["channels"]["rx_tp5"],
             rep["inputs"]["offset_model_declared"]))
    print("=" * 78)
    for grp in ("results_v3", "results_la"):
        for k, v in rep[grp].items():
            print("%-8s %-13s %s" % (k, v["status"], v["detail"]))
    for n in rep["notes"]:
        print("NOTE:", n)
    print("-" * 78)
    print("OVERALL (automated items): %s   [LA-10 is manual]" % rep["overall"])


def main(argv=None):
    ap = argparse.ArgumentParser(prog=TOOL_NAME, description=__doc__.split("\n\n")[0])
    sub = ap.add_subparsers(dest="cmd")
    p = sub.add_parser("info", help="container and edge statistics")
    p.add_argument("sr")
    p.add_argument("--compare", nargs=2, metavar=("CH_A", "CH_B"))
    p.add_argument("--dirb", nargs="+", metavar="CH",
                   help="decode these channels as Direction-B")
    p.add_argument("--out-json")
    p.add_argument("--progress", action="store_true")
    p = sub.add_parser("analyze", help="Formal analysis of one run")
    p.add_argument("--sr", required=True)
    p.add_argument("--evidence-json", required=True)
    p.add_argument("--tx", required=True, help="channel name of Main->TC TX")
    p.add_argument("--rx", required=True, help="channel name of TP5")
    p.add_argument("--offset-model", choices=["RAW", "WIRE_CORRECTED"],
                   help="model declared in the preflight (section 6.7)")
    p.add_argument("--tx-invert", action="store_true",
                   help="TX probed at the Q2 gate (TP4): polarity opposite to the "
                        "bus.  Default: TX is bus-side (TP3), idle HIGH / START LOW")
    p.add_argument("--formal", action="store_true")
    p.add_argument("--out-json")
    p.add_argument("--progress", action="store_true")
    sub.add_parser("hash", help="print this tool's sha256")
    ap.add_argument("--version", action="version",
                    version="%s %s" % (TOOL_NAME, TOOL_VERSION))
    a = ap.parse_args(argv)
    try:
        if a.cmd == "info":
            return cmd_info(a)
        if a.cmd == "analyze":
            return cmd_analyze(a)
        if a.cmd == "hash":
            print(tool_sha256())
            return 0
    except (OSError, ValueError, KeyError, zipfile.BadZipFile) as e:
        # input problems must never look like a test FAIL (exit code 1)
        sys.stderr.write("INPUT ERROR (%s): %s\n" % (type(e).__name__, e))
        return 3
    ap.print_help()
    return 3


if __name__ == "__main__":
    sys.exit(main())
