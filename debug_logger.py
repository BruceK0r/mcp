import atexit
import csv
import json
import os
import sys
import threading
import time
from datetime import datetime


class TeeStream:
    def __init__(self, primary_stream, log_file, lock):
        self.primary_stream = primary_stream
        self.log_file = log_file
        self.lock = lock
        self.encoding = getattr(primary_stream, "encoding", "utf-8")

    def write(self, text):
        with self.lock:
            self.primary_stream.write(text)
            self.log_file.write(text)
        return len(text)

    def flush(self):
        with self.lock:
            self.primary_stream.flush()
            self.log_file.flush()

    def isatty(self):
        return self.primary_stream.isatty()

    def fileno(self):
        return self.primary_stream.fileno()


class RuntimeDebugLogger:
    TRAJECTORY_FIELDS = [
        "wall_time",
        "elapsed_s",
        "cycle",
        "vehicle_name",
        "route_file",
        "controller",
        "state_valid",
        "route_ready",
        "status",
        "x",
        "y",
        "yaw_rad",
        "yaw_deg",
        "speed_mps",
        "cmd_v",
        "cmd_w",
        "nearest_index",
        "target_index",
        "target_x",
        "target_y",
        "cross_track_error",
        "heading_error_rad",
        "path_curvature",
        "calc_time_ms",
        "loop_time_ms",
    ]

    def __init__(self, base_dir="logs", enabled=True):
        self.enabled = enabled and os.environ.get("DEBUG_LOG_ENABLED", "1") != "0"
        self.base_dir = base_dir
        self.run_dir = None
        self.terminal_log_path = None
        self.trajectory_log_path = None
        self.metadata_path = None

        self._started_at = time.time()
        self._stdout = None
        self._stderr = None
        self._terminal_file = None
        self._trajectory_file = None
        self._trajectory_writer = None
        self._terminal_lock = threading.RLock()
        self._trajectory_lock = threading.RLock()
        self._closed = False

    def start(self):
        if not self.enabled:
            return self

        run_name = datetime.now().strftime("run_%Y%m%d_%H%M%S")
        self.run_dir = os.path.join(self.base_dir, run_name)
        os.makedirs(self.run_dir, exist_ok=True)

        self.terminal_log_path = os.path.join(self.run_dir, "terminal.log")
        self.trajectory_log_path = os.path.join(self.run_dir, "trajectory.csv")
        self.metadata_path = os.path.join(self.run_dir, "metadata.json")

        self._terminal_file = open(self.terminal_log_path, "a", encoding="utf-8", buffering=1)
        self._trajectory_file = open(self.trajectory_log_path, "a", encoding="utf-8", newline="", buffering=1)
        self._trajectory_writer = csv.DictWriter(self._trajectory_file, fieldnames=self.TRAJECTORY_FIELDS)
        if self._trajectory_file.tell() == 0:
            self._trajectory_writer.writeheader()

        self._stdout = sys.stdout
        self._stderr = sys.stderr
        sys.stdout = TeeStream(self._stdout, self._terminal_file, self._terminal_lock)
        sys.stderr = TeeStream(self._stderr, self._terminal_file, self._terminal_lock)

        atexit.register(self.close)
        print("debug log dir:", self.run_dir)
        return self

    def write_metadata(self, data):
        if not self.enabled or not self.metadata_path:
            return
        payload = {
            "started_at": datetime.fromtimestamp(self._started_at).isoformat(timespec="seconds"),
            "run_dir": self.run_dir,
        }
        payload.update(data or {})
        try:
            with open(self.metadata_path, "w", encoding="utf-8") as file:
                json.dump(payload, file, ensure_ascii=False, indent=2)
        except OSError as exc:
            print("failed to write debug metadata:", exc)

    def log_trajectory(self, **kwargs):
        if not self.enabled or self._trajectory_writer is None or self._closed:
            return

        now = time.time()
        row = {field: "" for field in self.TRAJECTORY_FIELDS}
        row["wall_time"] = datetime.fromtimestamp(now).isoformat(timespec="milliseconds")
        row["elapsed_s"] = self._format_float(now - self._started_at, 3)

        for key, value in kwargs.items():
            if key not in row:
                continue
            row[key] = self._format_value(value)

        with self._trajectory_lock:
            try:
                self._trajectory_writer.writerow(row)
            except ValueError:
                # File was closed during interpreter shutdown.
                pass

    def close(self):
        if self._closed:
            return
        self._closed = True

        if self._stdout is not None:
            sys.stdout = self._stdout
        if self._stderr is not None:
            sys.stderr = self._stderr

        for file_obj in (self._trajectory_file, self._terminal_file):
            if file_obj is None:
                continue
            try:
                file_obj.flush()
                file_obj.close()
            except OSError:
                pass

    def _format_value(self, value):
        if isinstance(value, float):
            return self._format_float(value)
        if isinstance(value, bool):
            return int(value)
        if value is None:
            return ""
        return value

    def _format_float(self, value, precision=6):
        try:
            value = float(value)
        except (TypeError, ValueError):
            return ""
        if value != value or value in (float("inf"), float("-inf")):
            return ""
        return f"{value:.{precision}f}"
