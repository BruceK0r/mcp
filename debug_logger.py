import csv
import atexit
import json
import math
import os
import time


class DebugLogger:
    def __init__(
        self,
        control_log_enabled=True,
        trajectory_log_enabled=True,
        log_dir="log",
        run_route_dir="run_route",
        log_every_n_frames=3,
        route_save_every_n_frames=3,
        enabled=None,
        interval=None,
    ):
        if enabled is not None:
            control_log_enabled = bool(enabled)

        self.control_log_enabled = bool(control_log_enabled)
        self.trajectory_log_enabled = bool(trajectory_log_enabled)
        self.log_dir = log_dir
        self.run_route_dir = run_route_dir
        self.log_every_n_frames = max(1, int(log_every_n_frames))
        self.route_save_every_n_frames = max(1, int(route_save_every_n_frames))
        self.frame = 0
        self._control_file = None
        self._control_writer = None
        self._trajectory_path = None
        self._trajectory_points = []
        self._closed = False

        stamp = time.strftime("%Y%m%d_%H%M%S")
        if self.control_log_enabled:
            os.makedirs(self.log_dir, exist_ok=True)
            self._control_file = open(
                os.path.join(self.log_dir, "control_%s.csv" % stamp),
                "w",
                newline="",
                encoding="utf-8",
                buffering=1,
            )
            self._control_writer = csv.DictWriter(self._control_file, fieldnames=self._control_fields())
            self._control_writer.writeheader()
        if self.trajectory_log_enabled:
            os.makedirs(self.run_route_dir, exist_ok=True)
            self._trajectory_path = os.path.join(self.run_route_dir, "run_route_%s.json" % stamp)
            self._save_trajectory()
        atexit.register(self.close)

    def log(self, data):
        self.frame += 1
        now = time.time()

        if self.trajectory_log_enabled and self._trajectory_path is not None:
            self._trajectory_points.append(
                {
                    "x": round(float(data.get("x", 0.0)), 6),
                    "y": round(float(data.get("y", 0.0)), 6),
                }
            )
            if self.frame % self.route_save_every_n_frames == 0:
                self._save_trajectory()

        if not self.control_log_enabled or self._control_writer is None:
            return
        if self.frame % self.log_every_n_frames != 0:
            return

        self._control_writer.writerow(
            {
                "timestamp": "%.6f" % now,
                "frame": self.frame,
                "x": self._fmt(data.get("x", 0.0)),
                "y": self._fmt(data.get("y", 0.0)),
                "yaw_rad": self._fmt(data.get("yaw", 0.0)),
                "yaw_deg": self._fmt(math.degrees(data.get("yaw", 0.0))),
                "speed": self._fmt(data.get("speed", 0.0)),
                "nearest_index": data.get("index", 0),
                "target_index": data.get("target_index", 0),
                "target_x": self._fmt(data.get("target_x", 0.0)),
                "target_y": self._fmt(data.get("target_y", 0.0)),
                "cross_track_error": self._fmt(data.get("cross_track_error", 0.0)),
                "heading_error": self._fmt(data.get("heading_error", 0.0)),
                "curvature": self._fmt(data.get("curvature", 0.0)),
                "target_speed": self._fmt(data.get("target_speed", 0.0)),
                "v_cmd": self._fmt(data.get("v_cmd", 0.0)),
                "w_cmd": self._fmt(data.get("w_cmd", 0.0)),
                "steering_angle": self._fmt(data.get("steering_angle", 0.0)),
            }
        )

    def close(self):
        if self._closed:
            return
        self._closed = True

        if self._control_file is not None and not self._control_file.closed:
            self._control_file.flush()
            self._control_file.close()

        if self.trajectory_log_enabled and self._trajectory_path is not None:
            self._save_trajectory()

    def __del__(self):
        self.close()

    def _fmt(self, value):
        return "%.6f" % float(value)

    def _save_trajectory(self):
        if self._trajectory_path is None:
            return
        tmp_path = self._trajectory_path + ".tmp"
        with open(tmp_path, "w", encoding="utf-8") as file_obj:
            json.dump(self._trajectory_points, file_obj, ensure_ascii=False, indent=2)
            file_obj.write("\n")
            file_obj.flush()
            os.fsync(file_obj.fileno())
        os.replace(tmp_path, self._trajectory_path)

    def _control_fields(self):
        return [
            "timestamp",
            "frame",
            "x",
            "y",
            "yaw_rad",
            "yaw_deg",
            "speed",
            "nearest_index",
            "target_index",
            "target_x",
            "target_y",
            "cross_track_error",
            "heading_error",
            "curvature",
            "target_speed",
            "v_cmd",
            "w_cmd",
            "steering_angle",
        ]
