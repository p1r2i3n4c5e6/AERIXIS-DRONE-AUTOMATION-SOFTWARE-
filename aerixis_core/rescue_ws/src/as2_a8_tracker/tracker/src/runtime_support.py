#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
===============================================================================
Shared Runtime Support — CPU Pinning, Logging, Crash Capture, Health Monitor
===============================================================================
Used by hybrid_tracker.py on BOTH the Raspberry Pi 5 (Linux) and the laptop
(Windows) so the two runs are directly comparable.

  pin_current_process()  : same core set on both platforms
                             Linux   -> os.sched_setaffinity(0, {...})
                             Windows -> SetProcessAffinityMask() via ctypes
  setup_logging()        : rotating file log + console + stdout/stderr tee
  install_crash_hooks()  : sys.excepthook, threading.excepthook, faulthandler
  ResourceMonitor        : periodic temp / freq / throttle / memory samples
  flush_and_fsync()      : forces log records to disk (survives a hard reboot)

Why fsync: a normal log flush only pushes data into the kernel page cache.
If the Pi 5 loses power (brown-out / undervoltage reboot), that cache is lost.
Calling fsync every few seconds bounds the loss to the sample interval.
===============================================================================
"""

from __future__ import annotations

import ctypes
import logging
import logging.handlers
import os
import platform
import shutil
import signal
import subprocess
import sys
import threading
import time
import traceback
from pathlib import Path

LOG_FORMAT = "%(asctime)s.%(msecs)03d %(levelname)-8s [%(name)s] %(message)s"
LOG_DATE = "%Y-%m-%d %H:%M:%S"

# Keep references alive for the lifetime of the process: faulthandler and the
# logging handlers both need their underlying file handles to stay referenced.
_crash_file_handle = None


# ─────────────────────────────────────────────────────────────────────────────
#  LOGGING
# ─────────────────────────────────────────────────────────────────────────────
class _TeeStream:
    """Mirror every print() into the log while still writing to the console."""

    def __init__(self, stream, logger, level=logging.INFO):
        self._stream = stream
        self._logger = logger
        self._level = level
        self._buf = ""

    def write(self, data):
        if not isinstance(data, str):
            data = str(data)
        if self._stream is not None:
            try:
                self._stream.write(data)
            except Exception:
                pass          # a broken console must never kill the pipeline
        self._buf += data
        while "\n" in self._buf:
            line, self._buf = self._buf.split("\n", 1)
            if line.strip():
                self._logger.log(self._level, line.rstrip("\r"))
        return len(data)

    def flush(self):
        if self._stream is not None:
            try:
                self._stream.flush()
            except Exception:
                pass
        if self._buf.strip():
            self._logger.log(self._level, self._buf.strip())
            self._buf = ""

    def fileno(self):
        return self._stream.fileno() if self._stream is not None else -1

    def isatty(self):
        return bool(getattr(self._stream, "isatty", lambda: False)())

    @property
    def encoding(self):
        return getattr(self._stream, "encoding", "utf-8")


def _reconfigure_stream(stream):
    """Force UTF-8 so the emoji banners cannot raise UnicodeEncodeError on cp1252."""
    try:
        stream.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass


class _DropTeeEcho(logging.Filter):
    """Console handler filter: the tee already printed these lines to the console."""

    _TEE_LOGGERS = {"stdout", "stderr"}

    def filter(self, record):
        return record.name not in self._TEE_LOGGERS


def setup_logging(log_dir="logs", log_name="hybrid_tracker",
                  level=logging.INFO, console=True,
                  max_bytes=10 * 1024 * 1024, backups=10):
    """
    Configure the root logger: rotating UTF-8 file + console, and tee stdout.

    Every log record is flushed on emit, and ResourceMonitor additionally
    fsyncs the file, so the last lines survive a power-cut reboot.

    Returns the log file path.
    """
    log_path = Path(log_dir).expanduser().resolve()
    log_path.mkdir(parents=True, exist_ok=True)
    file_path = log_path / f"{log_name}.log"

    root = logging.getLogger()
    root.setLevel(level)
    for handler in list(root.handlers):
        root.removeHandler(handler)

    file_handler = logging.handlers.RotatingFileHandler(
        file_path, maxBytes=max_bytes, backupCount=backups,
        encoding="utf-8", delay=False,
    )
    file_handler.setFormatter(logging.Formatter(LOG_FORMAT, LOG_DATE))
    root.addHandler(file_handler)

    if console:
        _reconfigure_stream(sys.stdout)
        _reconfigure_stream(sys.stderr)
        console_handler = logging.StreamHandler(sys.stderr)
        console_handler.setFormatter(logging.Formatter(LOG_FORMAT, LOG_DATE))
        console_handler.addFilter(_DropTeeEcho())
        root.addHandler(console_handler)

        # Tee AFTER the console handler captured the real streams, so a
        # print() -> tee -> logger -> handler write cannot recurse.
        sys.stdout = _TeeStream(sys.stdout, logging.getLogger("stdout"), logging.INFO)
        sys.stderr = _TeeStream(sys.stderr, logging.getLogger("stderr"), logging.WARNING)

    log = logging.getLogger("runtime")
    log.info("=" * 78)
    log.info("SESSION START  pid=%d  python=%s", os.getpid(), platform.python_version())
    log.info("platform  = %s", platform.platform())
    log.info("machine   = %s  cpus=%d", platform.machine(), os.cpu_count() or 0)
    log.info("cwd       = %s", os.getcwd())
    log.info("argv      = %s", " ".join(sys.argv))
    log.info("logfile   = %s", file_path)
    log.info("=" * 78)
    return file_path


def flush_and_fsync():
    """Push buffered log records all the way to disk (page cache -> platter/NAND)."""
    for handler in logging.getLogger().handlers:
        try:
            handler.flush()
        except Exception:
            continue
        stream = getattr(handler, "stream", None)
        if stream is None or not hasattr(stream, "fileno"):
            continue
        try:
            os.fsync(stream.fileno())
        except Exception:
            pass


# ─────────────────────────────────────────────────────────────────────────────
#  CRASH CAPTURE
# ─────────────────────────────────────────────────────────────────────────────
def install_crash_hooks(log_dir="logs", log_name="hybrid_tracker"):
    """
    Log every uncaught exception, from the main thread and from worker threads,
    and arm faulthandler for native crashes (SIGSEGV in OpenCV/NCNN/FFmpeg).
    """
    global _crash_file_handle

    log = logging.getLogger("crash")
    crash_path = Path(log_dir).expanduser().resolve() / f"{log_name}_crash.log"
    crash_path.parent.mkdir(parents=True, exist_ok=True)

    try:
        import faulthandler
        _crash_file_handle = open(crash_path, "a", encoding="utf-8")
        _crash_file_handle.write(
            f"\n=== faulthandler armed {time.strftime('%Y-%m-%d %H:%M:%S')} "
            f"pid={os.getpid()} argv={' '.join(sys.argv)} ===\n")
        _crash_file_handle.flush()
        faulthandler.enable(file=_crash_file_handle, all_threads=True)
    except Exception as exc:
        log.warning("faulthandler could not be armed: %s", exc)

    def _excepthook(exc_type, exc, tb):
        if issubclass(exc_type, KeyboardInterrupt):
            sys.__excepthook__(exc_type, exc, tb)
            return
        log.critical("UNCAUGHT EXCEPTION (main thread)\n%s",
                     "".join(traceback.format_exception(exc_type, exc, tb)))
        flush_and_fsync()

    sys.excepthook = _excepthook

    def _thread_excepthook(args):
        if args.exc_type is SystemExit:
            return
        log.critical("UNCAUGHT EXCEPTION (thread '%s')\n%s",
                     getattr(args.thread, "name", "?"),
                     "".join(traceback.format_exception(
                         args.exc_type, args.exc_value, args.exc_traceback)))
        flush_and_fsync()

    threading.excepthook = _thread_excepthook

    # Graceful shutdown on SIGTERM / Ctrl-Break so the finally: block still runs.
    def _signal_handler(signum, _frame):
        try:
            name = signal.Signals(signum).name
        except Exception:
            name = str(signum)
        log.warning("Received %s (%s) — shutting down", name, signum)
        raise SystemExit(128 + int(signum))

    for sig_name in ("SIGTERM", "SIGHUP", "SIGBREAK"):
        sig = getattr(signal, sig_name, None)
        if sig is None:
            continue
        try:
            signal.signal(sig, _signal_handler)
        except (ValueError, OSError):
            pass          # not available in this context (e.g. non-main thread)

    return crash_path


# ─────────────────────────────────────────────────────────────────────────────
#  CPU PINNING
# ─────────────────────────────────────────────────────────────────────────────
def pin_current_process(cores, logger=None):
    """
    Pin this process (and every thread it creates afterwards) to `cores`.

    Returns True when the affinity mask was applied and read back successfully.
    CPU indices that do not exist on this machine are dropped with a warning so
    the same command line works on the 4-core Pi 5 and on a laptop.
    """
    log = logger or logging.getLogger("cpu")
    requested = sorted({int(c) for c in cores})
    if not requested:
        log.warning("[CPU Isolation] No cores requested — running unpinned.")
        return False

    total = os.cpu_count() or 1
    valid = [c for c in requested if 0 <= c < total]
    dropped = [c for c in requested if c not in valid]
    if dropped:
        log.warning("[CPU Isolation] Ignoring cores %s — this machine has %d CPUs (0-%d).",
                    dropped, total, total - 1)
    if not valid:
        log.error("[CPU Isolation] None of the requested cores exist here — running unpinned.")
        return False

    # ── Linux (Raspberry Pi 5) ───────────────────────────────────────────────
    if sys.platform.startswith("linux") and hasattr(os, "sched_setaffinity"):
        try:
            os.sched_setaffinity(0, set(valid))
            applied = sorted(os.sched_getaffinity(0)) if hasattr(os, "sched_getaffinity") else valid
            log.info("[CPU Isolation] Pinned to cores %s (requested %s).", applied, requested)
            return True
        except OSError as exc:
            log.error("[CPU Isolation] sched_setaffinity(%s) failed: %s", valid, exc)
            return False

    # ── Windows (laptop) ─────────────────────────────────────────────────────
    if os.name == "nt":
        try:
            k32 = ctypes.WinDLL("kernel32", use_last_error=True)
            k32.GetCurrentProcess.restype = ctypes.c_void_p
            k32.GetCurrentProcess.argtypes = []
            k32.SetProcessAffinityMask.argtypes = [ctypes.c_void_p, ctypes.c_size_t]
            k32.SetProcessAffinityMask.restype = ctypes.c_int
            k32.GetProcessAffinityMask.argtypes = [
                ctypes.c_void_p,
                ctypes.POINTER(ctypes.c_size_t),
                ctypes.POINTER(ctypes.c_size_t),
            ]
            k32.GetProcessAffinityMask.restype = ctypes.c_int

            mask = 0
            for core in valid:
                mask |= (1 << core)

            handle = k32.GetCurrentProcess()
            if not k32.SetProcessAffinityMask(handle, mask):
                raise OSError(ctypes.get_last_error(), "SetProcessAffinityMask failed")

            # Read it back: what was granted, not what was asked for.
            proc_mask = ctypes.c_size_t(0)
            sys_mask = ctypes.c_size_t(0)
            k32.GetProcessAffinityMask(handle, ctypes.byref(proc_mask), ctypes.byref(sys_mask))
            applied = [c for c in range(64) if proc_mask.value & (1 << c)]
            log.info("[CPU Isolation] Pinned to cores %s (mask 0x%X, requested %s).",
                     applied, proc_mask.value, requested)
            return True
        except Exception as exc:
            log.error("[CPU Isolation] Windows SetProcessAffinityMask failed: %s", exc)
            return False

    log.warning("[CPU Isolation] Affinity pinning unsupported on %s — running unpinned.",
                sys.platform)
    return False


def configure_thread_env(thread_count, logger=None):
    """
    Cap every math/vision thread pool at `thread_count`.
    NCNN, OpenBLAS, OpenMP and OpenCV each spawn their own pools; without this
    the process can use more worker threads than the Pi run does, which makes
    laptop timings non-comparable and defeats the core pinning.
    """
    log = logger or logging.getLogger("cpu")
    n = max(1, int(thread_count))
    for var in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS",
                "NUMEXPR_NUM_THREADS", "VECLIB_MAXIMUM_THREADS"):
        os.environ[var] = str(n)
    try:
        import cv2
        cv2.setNumThreads(n)
    except Exception:
        pass
    log.info("[Thread Pool] OMP/OpenBLAS/OpenCV capped at %d thread(s).", n)


# ─────────────────────────────────────────────────────────────────────────────
#  RESOURCE / HEALTH MONITOR
# ─────────────────────────────────────────────────────────────────────────────
_THROTTLE_BITS = (
    (0x00001, "UNDER-VOLTAGE NOW"),
    (0x00002, "ARM FREQUENCY CAPPED NOW"),
    (0x00004, "THROTTLED NOW"),
    (0x00008, "SOFT TEMPERATURE LIMIT NOW"),
    (0x10000, "under-voltage has occurred"),
    (0x20000, "arm frequency capping has occurred"),
    (0x40000, "throttling has occurred"),
    (0x80000, "soft temperature limit has occurred"),
)


def decode_throttled(value: int) -> list:
    return [label for bit, label in _THROTTLE_BITS if value & bit]


def _read_cpu_temp_c():
    """Hottest thermal zone in °C, or None when unavailable."""
    base = Path("/sys/class/thermal")
    if not base.is_dir():
        return None
    temps = []
    for zone in base.glob("thermal_zone*/temp"):
        try:
            temps.append(int(zone.read_text().strip()) / 1000.0)
        except Exception:
            continue
    return max(temps) if temps else None


def _read_cpu_freq_mhz():
    """Highest current ARM core frequency in MHz, or None."""
    base = Path("/sys/devices/system/cpu")
    if not base.is_dir():
        return None
    freqs = []
    for f in base.glob("cpu[0-9]*/cpufreq/scaling_cur_freq"):
        try:
            freqs.append(int(f.read_text().strip()) / 1000.0)
        except Exception:
            continue
    return max(freqs) if freqs else None


def _read_throttled():
    if shutil.which("vcgencmd") is None:
        return None
    try:
        out = subprocess.run(["vcgencmd", "get_throttled"], capture_output=True,
                             text=True, timeout=2.0).stdout.strip()
        return int(out.split("=", 1)[1], 16)
    except Exception:
        return None


def _read_mem_available_mb():
    if sys.platform.startswith("linux"):
        try:
            for line in Path("/proc/meminfo").read_text().splitlines():
                if line.startswith("MemAvailable:"):
                    return int(line.split()[1]) / 1024.0
        except Exception:
            return None
    if os.name == "nt":
        class _MEMORYSTATUSEX(ctypes.Structure):
            _fields_ = [
                ("dwLength", ctypes.c_ulong),
                ("dwMemoryLoad", ctypes.c_ulong),
                ("ullTotalPhys", ctypes.c_ulonglong),
                ("ullAvailPhys", ctypes.c_ulonglong),
                ("ullTotalPageFile", ctypes.c_ulonglong),
                ("ullAvailPageFile", ctypes.c_ulonglong),
                ("ullTotalVirtual", ctypes.c_ulonglong),
                ("ullAvailVirtual", ctypes.c_ulonglong),
                ("ullAvailExtendedVirtual", ctypes.c_ulonglong),
            ]
        try:
            stat = _MEMORYSTATUSEX()
            stat.dwLength = ctypes.sizeof(_MEMORYSTATUSEX)
            if ctypes.windll.kernel32.GlobalMemoryStatusEx(ctypes.byref(stat)):
                return stat.ullAvailPhys / (1024.0 * 1024.0)
        except Exception:
            return None
    return None


def _read_process_rss_mb():
    try:
        import psutil
        return psutil.Process().memory_info().rss / (1024.0 * 1024.0)
    except Exception:
        pass
    try:
        pages = int(Path("/proc/self/statm").read_text().split()[1])
        return pages * os.sysconf("SC_PAGE_SIZE") / (1024.0 * 1024.0)
    except Exception:
        return None


class ResourceMonitor(threading.Thread):
    """
    Samples temperature, frequency, memory and the Pi's throttle register every
    `interval` seconds and fsyncs the log right after.

    Undervoltage bits (0x1 / 0x10000) are the smoking gun for a Pi 5 that
    reboots under vision load — they are written to disk BEFORE the reboot,
    so the reason survives the power cut.
    """

    TEMP_WARN_C = 80.0

    def __init__(self, interval=5.0, logger=None):
        super().__init__(name="resource-monitor", daemon=True)
        self.interval = float(interval)
        self.log = logger or logging.getLogger("monitor")
        self._stop_event = threading.Event()

    def stop(self):
        self._stop_event.set()

    def run(self):
        while not self._stop_event.wait(self.interval):
            try:
                self._sample()
            except Exception:
                self.log.exception("Resource monitor sample failed")
            finally:
                flush_and_fsync()

    def _sample(self):
        parts = []

        temp = _read_cpu_temp_c()
        if temp is not None:
            parts.append(f"temp={temp:.1f}C")

        freq = _read_cpu_freq_mhz()
        if freq is not None:
            parts.append(f"cpu={freq:.0f}MHz")

        try:
            load = os.getloadavg()
            parts.append(f"load={load[0]:.2f}/{load[1]:.2f}/{load[2]:.2f}")
        except Exception:
            pass

        mem = _read_mem_available_mb()
        if mem is not None:
            parts.append(f"mem_avail={mem:.0f}MB")

        rss = _read_process_rss_mb()
        if rss is not None:
            parts.append(f"rss={rss:.0f}MB")

        parts.append(f"python_threads={threading.active_count()}")
        try:
            import psutil
            parent = psutil.Process()
            processes = [parent] + parent.children(recursive=True)
            total_rss = total_threads = count = 0
            for process in processes:
                try:
                    total_rss += process.memory_info().rss
                    total_threads += process.num_threads()
                    count += 1
                except psutil.Error:
                    pass  # A worker may finish during the sample.
            parts.append(f"processes={count} native_threads_total={total_threads} "
                         f"rss_sum={total_rss/(1024.*1024.):.0f}MB")
        except (ImportError, OSError):
            pass

        throttled = _read_throttled()
        if throttled is not None:
            parts.append(f"throttled=0x{throttled:05X}")
            flags = decode_throttled(throttled)
            if flags:
                # Undervoltage / throttling is the usual reason a Pi 5 reboots
                # under vision load. Write it to disk BEFORE any reboot.
                self.log.warning("[Health] vcgencmd throttled=0x%05X -> %s  (%s)",
                                 throttled, ", ".join(flags), "  ".join(parts))
                flush_and_fsync()
                return

        line = "  ".join(parts)
        if temp is not None and temp >= self.TEMP_WARN_C:
            self.log.warning("[Health] CPU at %.1fC (>=%.0fC)  %s",
                             temp, self.TEMP_WARN_C, line)
        else:
            self.log.info("[Health] %s", line)


def get_logger(name="runtime"):
    return logging.getLogger(name)
