# NTC_1A_rxlog_v3_6_beta.py - v3.6 BETA (NOT PRODUCTION)
# Receive/transmit evidence logger for the Pi GUI (GUI v3.6 BETA).
#
# Purpose:
#   Keep a field-reviewable record of what the Pi actually received from the
#   ESP32 (raw bytes, parsed Direction-B frames, RX status changes, dropped
#   bytes) and sent (SEND / RESET / SENS.ADJ ...), plus operator marks, so a
#   remote site test can be analysed offline.
#
# Design rules:
#   - Never block or slow the RX/TX paths: the hook only does put_nowait on a
#     bounded queue. If the queue is full the event is counted and discarded.
#   - Never raise into the caller.
#   - Raw bytes are logged before any interpretation, so a parser defect can
#     not lose evidence.
#   - Append-only CSV (UTF-8, Excel friendly). Size-based rotation. Only files
#     created by this module (prefix "rxlog_") are ever pruned.
#   - No control-path behaviour is changed by this module.
#
# CSV columns:
#   wall_time, mono_s, event, hex, wind_length, actual_tension, rx_status, detail
#
# Events:
#   START      one line per file: revisions, port, host, python
#   RX_RAW     bytes of one serial read (chunk), before framing
#   RX_FRAME   valid Direction-B frame (hex + parsed values + dt since previous)
#   RX_BOOT    ESP32 Boot Magic frame
#   RX_LEGACY  legacy checksum7 frame (12/8 bytes)
#   RX_DROP    one byte discarded during resynchronisation (noise indicator)
#   RX_ERR     serial read error
#   TX / TX_ERR  command bytes sent by the Pi
#   STATUS     RX status transition (WAIT/OK/LOST)
#   NOTE       operator mark (RUN / STOP / MARK ...)
#   PORT       port open/close
#   SUMMARY    cumulative counters every SUMMARY_INTERVAL_SEC
#   STOP       logger stopped

import csv
import datetime
import os
import platform
import queue
import shutil
import threading
import time
from pathlib import Path

REVISION = "v3.6-beta"

DEFAULT_LOG_DIR = Path.home() / "ntc_logs"
FILE_PREFIX = "rxlog_"
MAX_FILE_BYTES = 10 * 1024 * 1024          # rotate at 10 MB
MAX_TOTAL_BYTES = 500 * 1024 * 1024        # prune oldest rxlog_ files above 500 MB
MIN_FREE_BYTES = 200 * 1024 * 1024         # stop writing below this free space
QUEUE_MAX = 20000
FLUSH_INTERVAL_SEC = 2.0
FSYNC_INTERVAL_SEC = 30.0
SUMMARY_INTERVAL_SEC = 60.0
STATUS_POLL_SEC = 0.2

COLUMNS = ("wall_time", "mono_s", "event", "hex",
           "wind_length", "actual_tension", "rx_status", "detail")

_KIND_TO_EVENT = {
    "rx_raw": "RX_RAW",
    "rx_frame": "RX_FRAME",
    "rx_boot": "RX_BOOT",
    "rx_legacy": "RX_LEGACY",
    "rx_drop": "RX_DROP",
    "rx_err": "RX_ERR",
    "tx": "TX",
    "tx_err": "TX_ERR",
    "port_open": "PORT",
    "port_close": "PORT",
}


def _hex(data):
    return " ".join(f"{b:02X}" for b in data)


class RxLogger:
    def __init__(self, log_dir=None, meta=None, get_status=None,
                 max_file_bytes=MAX_FILE_BYTES, max_total_bytes=MAX_TOTAL_BYTES,
                 min_free_bytes=MIN_FREE_BYTES, queue_max=QUEUE_MAX):
        self.log_dir = Path(log_dir) if log_dir else DEFAULT_LOG_DIR
        self.meta = dict(meta or {})
        self.get_status = get_status
        self.max_file_bytes = max_file_bytes
        self.max_total_bytes = max_total_bytes
        self.min_free_bytes = min_free_bytes

        self._q = queue.Queue(maxsize=queue_max)
        self._stop = threading.Event()
        self._writer = None
        self._watcher = None

        self._t0 = time.monotonic()
        self._file = None
        self._csv = None
        self._path = None
        self._part = 0
        self._stamp = None
        self._last_fsync = 0.0
        self._last_flush = 0.0
        self._wrote_since_flush = False

        # counters (touched by writer thread, read by GUI for display)
        self.lines_written = 0
        self.overflow_dropped = 0
        self.write_error = None
        self.disabled_reason = None
        self.frames = 0
        self.drops = 0
        self.raw_bytes = 0
        self.tx_count = 0
        self.status_changes = 0
        self._last_frame_mono = None
        self._interval_min_dt = None
        self._interval_max_dt = None
        self._last_status = None
        self._notes = 0

    # ------------------------------------------------------------------
    # public API
    # ------------------------------------------------------------------
    @property
    def path(self):
        return self._path

    def start(self):
        try:
            self.log_dir.mkdir(parents=True, exist_ok=True)
            self._open_new_file()
        except Exception as e:  # logging unavailable must not stop the GUI
            self.write_error = str(e)
            self.disabled_reason = f"open failed: {e}"
            return False
        self._writer = threading.Thread(target=self._writer_loop, name="rxlog-writer", daemon=True)
        self._writer.start()
        if self.get_status is not None:
            self._watcher = threading.Thread(target=self._watch_loop, name="rxlog-status", daemon=True)
            self._watcher.start()
        return True

    def stop(self):
        self._enqueue("STOP", detail=self._counters_text())
        self._stop.set()
        if self._writer and self._writer.is_alive():
            self._writer.join(timeout=2.0)
        self._close_file()

    def hook(self, kind, data, info):
        """Callback for NTC_1A_serial_comm_v3_6_beta.set_log_hook()."""
        event = _KIND_TO_EVENT.get(kind)
        if event is None:
            return
        now = time.monotonic()
        detail = ""
        wind = tens = ""
        if kind == "rx_raw":
            if not data:
                return
            self.raw_bytes += len(data)
            detail = f"len={len(data)}"
        elif kind == "rx_frame":
            self.frames += 1
            wind = info.get("wind_length", "")
            tens = info.get("actual_tension", "")
            if self._last_frame_mono is not None:
                dt_ms = (now - self._last_frame_mono) * 1000.0
                detail = f"dt={dt_ms:.0f}ms"
                if self._interval_min_dt is None or dt_ms < self._interval_min_dt:
                    self._interval_min_dt = dt_ms
                if self._interval_max_dt is None or dt_ms > self._interval_max_dt:
                    self._interval_max_dt = dt_ms
            self._last_frame_mono = now
        elif kind == "rx_drop":
            self.drops += 1
            detail = "resync"
        elif kind in ("tx", "tx_err"):
            self.tx_count += 1
            detail = f"label={info.get('label', '')}"
            if kind == "tx_err":
                detail += f" error={info.get('error', '')}"
        elif kind == "rx_err":
            detail = f"error={info.get('error', '')}"
        elif kind == "port_open":
            detail = f"open {info.get('port', '')}"
        elif kind == "port_close":
            detail = "close"
        self._enqueue(event, hex_=_hex(data), wind=wind, tens=tens, detail=detail, mono=now)

    def note(self, text):
        self._notes += 1
        self._enqueue("NOTE", detail=str(text))

    def status_text(self):
        """Short text for the GUI."""
        if self.disabled_reason:
            return "LOG ERR"
        return f"LOG {self.lines_written}"

    @property
    def healthy(self):
        return not self.disabled_reason and not self.write_error

    # ------------------------------------------------------------------
    # internals
    # ------------------------------------------------------------------
    def _enqueue(self, event, hex_="", wind="", tens="", detail="", mono=None, status=""):
        row = (datetime.datetime.now().isoformat(timespec="milliseconds"),
               f"{(mono if mono is not None else time.monotonic()) - self._t0:.3f}",
               event, hex_, wind, tens, status, detail)
        try:
            self._q.put_nowait(row)
        except queue.Full:
            self.overflow_dropped += 1

    def _counters_text(self):
        return (f"frames={self.frames} drops={self.drops} raw_bytes={self.raw_bytes} "
                f"tx={self.tx_count} status_changes={self.status_changes} "
                f"notes={self._notes} overflow={self.overflow_dropped} "
                f"lines={self.lines_written}")

    def _watch_loop(self):
        next_summary = time.monotonic() + SUMMARY_INTERVAL_SEC
        while not self._stop.wait(STATUS_POLL_SEC):
            try:
                status = self.get_status()
            except Exception:
                status = None
            if status is not None and status != self._last_status:
                prev = self._last_status
                self._last_status = status
                if prev is not None:
                    self.status_changes += 1
                self._enqueue("STATUS", status=status, detail=f"prev={prev}")
            now = time.monotonic()
            if now >= next_summary:
                next_summary = now + SUMMARY_INTERVAL_SEC
                extra = ""
                if self._interval_min_dt is not None:
                    extra = (f" interval_dt_min={self._interval_min_dt:.0f}ms"
                             f" interval_dt_max={self._interval_max_dt:.0f}ms")
                    self._interval_min_dt = None
                    self._interval_max_dt = None
                self._enqueue("SUMMARY", detail=self._counters_text() + extra)

    def _open_new_file(self):
        self._close_file()
        if self._stamp is None:
            self._stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self._part += 1
        self._path = self.log_dir / f"{FILE_PREFIX}{self._stamp}_part{self._part:02d}.csv"
        self._file = open(self._path, "w", newline="", encoding="utf-8")
        self._csv = csv.writer(self._file)
        self._csv.writerow(COLUMNS)
        meta = " ".join(f"{k}={v}" for k, v in self.meta.items())
        meta += f" rxlog={REVISION} host={platform.node()} python={platform.python_version()} part={self._part}"
        self._csv.writerow((datetime.datetime.now().isoformat(timespec="milliseconds"),
                            f"{time.monotonic() - self._t0:.3f}", "START", "", "", "", "", meta.strip()))
        self._file.flush()

    def _close_file(self):
        f, self._file, self._csv = self._file, None, None
        if f:
            try:
                f.flush()
                os.fsync(f.fileno())
            except Exception:
                pass
            try:
                f.close()
            except Exception:
                pass

    def _prune_old_files(self):
        """Delete only our own oldest rxlog_ files beyond the total-size cap."""
        try:
            files = sorted(self.log_dir.glob(f"{FILE_PREFIX}*.csv"), key=lambda p: p.stat().st_mtime)
            total = sum(p.stat().st_size for p in files)
            for p in files:
                if total <= self.max_total_bytes or p == self._path:
                    break
                size = p.stat().st_size
                p.unlink()
                total -= size
        except Exception:
            pass

    def _writer_loop(self):
        while True:
            try:
                row = self._q.get(timeout=0.5)
            except queue.Empty:
                row = None
            if row is not None and not self.disabled_reason:
                self._write_row(row)
            now = time.monotonic()
            if self._file and self._wrote_since_flush and now - self._last_flush >= FLUSH_INTERVAL_SEC:
                self._flush(now)
            if self._stop.is_set() and self._q.empty():
                break
        if self._file:
            self._flush(time.monotonic(), force_sync=True)

    def _write_row(self, row):
        try:
            if self._file.tell() >= self.max_file_bytes:
                free = shutil.disk_usage(self.log_dir).free
                if free < self.min_free_bytes:
                    self.disabled_reason = f"low disk space ({free // (1024 * 1024)} MB free)"
                    return
                self._prune_old_files()
                self._open_new_file()
            self._csv.writerow(row)
            self.lines_written += 1
            self._wrote_since_flush = True
        except Exception as e:
            self.write_error = str(e)
            self.disabled_reason = f"write failed: {e}"

    def _flush(self, now, force_sync=False):
        try:
            self._file.flush()
            self._wrote_since_flush = False
            self._last_flush = now
            if force_sync or now - self._last_fsync >= FSYNC_INTERVAL_SEC:
                os.fsync(self._file.fileno())
                self._last_fsync = now
        except Exception as e:
            self.write_error = str(e)
            self.disabled_reason = f"flush failed: {e}"
