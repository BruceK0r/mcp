import argparse
import logging
import math
import os
import signal
import time

from controller import ControllerConfig, PIDSpeedController, PurePursuitController
from debug_logger import DebugLogger
from my_udp import UDPClient
from optimizer import MPCLikeOptimizer
from path_utils import (
    SpeedProfileConfig,
    build_reference_trajectory,
    clamp,
    load_route_json,
    nearest_index,
    resample_polyline,
    sample_reference_at_s,
)
from planner import HybridAStarPlanner


DEFAULT_NET_CONFIG = "RST6phjHTlYisx4dqWmCnNpkYfTV,127.0.0.1,8448,8449"
DEFAULT_ROUTE_FILE = "exp_routes/Big.json"
_DEFAULT_DEBUG_LOGGER = object()


class Control:
    def __init__(
        self,
        route_file=None,
        udp_client=None,
        debug_logger=_DEFAULT_DEBUG_LOGGER,
        net_config=None,
        control_log_enabled=True,
        trajectory_log_enabled=True,
        log_dir="log",
        run_route_dir="run_route",
        log_every_n_frames=3,
        route_save_every_n_frames=3,
    ):
        self.config = ControllerConfig()
        self.vehicle_name = "1"
        self.udp_port = 9000
        self.udp_send_port = 9001
        self.server_ip = "192.168.1.100"
        self.route_file = route_file or os.environ.get("ROUTE_FILE", DEFAULT_ROUTE_FILE)

        selected_net_config = net_config if net_config is not None else os.environ.get("NET_CONFIG", DEFAULT_NET_CONFIG)
        self._load_network_config(selected_net_config)
        self.udp_client = udp_client or UDPClient(
            self.server_ip,
            self.udp_port,
            self.udp_send_port,
            self.vehicle_name,
        )
        if hasattr(self.udp_client, "logger"):
            self.udp_client.logger.setLevel(logging.WARNING)

        if debug_logger is _DEFAULT_DEBUG_LOGGER:
            self.debug_logger = DebugLogger(
                control_log_enabled=control_log_enabled,
                trajectory_log_enabled=trajectory_log_enabled,
                log_dir=log_dir,
                run_route_dir=run_route_dir,
                log_every_n_frames=log_every_n_frames,
                route_save_every_n_frames=route_save_every_n_frames,
            )
        else:
            self.debug_logger = debug_logger
        self.planner = HybridAStarPlanner(
            wheel_base=self.config.wheel_base,
            max_steer=self.config.max_steer,
            allow_reverse=False,
        )
        self.optimizer = MPCLikeOptimizer(
            iterations=self.config.smoothing_iterations,
            data_weight=self.config.smoothing_data_weight,
            smooth_weight=self.config.smoothing_smooth_weight,
            curvature_weight=self.config.smoothing_curvature_weight,
            max_step=self.config.smoothing_max_step,
        )
        self.speed_controller = PIDSpeedController(self.config)
        self.lateral_controller = PurePursuitController(self.config)

        self.m_v = 0.0
        self.m_x = 0.0
        self.m_y = 0.0
        self.m_yaw = 0.0
        self.vehpos_initial_index = 0
        self.num_preview = 8
        self.targetPos_Info = [0.0, 0.0]
        self.Y_points = []
        self.X_points = []
        self.control_rate = self.config.control_rate
        self.wheel_base = self.config.wheel_base
        self.trajectory = None
        self.path_length = 0.0
        self.last_target_index = 0
        self.last_target_speed = self.config.min_speed
        self.last_v_cmd = 0.0
        self.last_w_cmd = 0.0
        self.last_steering_angle = 0.0

    def _load_network_config(self, net_config):
        if not net_config:
            return
        parts = [part.strip() for part in net_config.split(",")]
        if len(parts) != 4:
            raise ValueError("NET_CONFIG must be 'vehicle_name,server_ip,udp_port,udp_send_port'.")
        self.vehicle_name = parts[0]
        self.server_ip = parts[1]
        self.udp_port = int(parts[2])
        self.udp_send_port = int(parts[3])

    def control_node(self):
        self.load_route(self.route_file)
        period = 1.0 / self.control_rate
        last_time = time.time()

        while True:
            loop_start = time.time()
            dt = max(loop_start - last_time, period)
            last_time = loop_start

            vehicle_data = self.udp_client.get_vehicle_state()
            self.m_x = float(vehicle_data.x)
            self.m_y = float(vehicle_data.y)
            self.m_yaw = math.radians(float(vehicle_data.yaw))
            self.m_v = max(0.0, float(vehicle_data.speed))

            v_cmd, w_cmd = self.compute_control(dt)
            self.udp_client.send_control_command(v_cmd, w_cmd)

            sleep_time = max(period - (time.time() - loop_start), 0.0)
            time.sleep(sleep_time)

    def load_route(self, file_path):
        raw_x, raw_y, closed = load_route_json(file_path)
        fallback_path = list(zip(raw_x, raw_y))

        coarse_path = self.planner.plan(
            start=None,
            goal=None,
            grid_map=None,
            fallback_path=fallback_path,
        )
        coarse_x = [p[0] for p in coarse_path]
        coarse_y = [p[1] for p in coarse_path]

        resampled_x, resampled_y = resample_polyline(
            coarse_x,
            coarse_y,
            spacing=self.config.resample_spacing,
            closed=closed,
        )
        smooth_x, smooth_y = self.optimizer.optimize(resampled_x, resampled_y, closed=closed)

        speed_config = SpeedProfileConfig(
            max_speed=self._speed_limit(),
            min_speed=self.config.min_speed,
            max_accel=self.config.max_accel,
            max_decel=self.config.max_decel,
            max_lateral_accel=self.config.max_lateral_accel,
            curvature_speed_gain=self.config.curvature_speed_gain,
            speed_filter_alpha=self.config.speed_filter_alpha,
        )
        self.trajectory = build_reference_trajectory(smooth_x, smooth_y, closed, speed_config)
        self.path_length = self.trajectory.length
        self.X_points = [point.x for point in self.trajectory.points]
        self.Y_points = [point.y for point in self.trajectory.points]

        if not self.X_points:
            raise ValueError("Reference trajectory is empty after processing.")

        self.search_vehicle_initial_index()
        return True

    def compute_control(self, dt):
        if self.trajectory is None or len(self.trajectory.points) == 0:
            self.load_route(self.route_file)

        self.update_vehpos_index()
        nearest = self.trajectory.points[self.vehpos_initial_index]
        self.last_target_speed = clamp(nearest.target_speed, 0.0, self._speed_limit())
        v_cmd = self.speed_controller.update(self.last_target_speed, self.m_v, dt)
        v_cmd = clamp(v_cmd, 0.0, self._speed_limit())

        lateral = self.lateral_controller.compute(
            self.trajectory,
            self.vehpos_initial_index,
            self.m_x,
            self.m_y,
            self.m_yaw,
            self.m_v,
            v_cmd,
            dt,
        )
        w_cmd = clamp(lateral.w_cmd, -self.config.max_yaw_rate, self.config.max_yaw_rate)

        self.targetPos_Info[0] = lateral.target_x
        self.targetPos_Info[1] = lateral.target_y
        self.last_target_index = lateral.target_index
        self.last_v_cmd = v_cmd
        self.last_w_cmd = w_cmd
        self.last_steering_angle = lateral.steering_angle

        self._log_debug(nearest, lateral, v_cmd, w_cmd)
        return v_cmd, w_cmd

    def calc_pure_pursuit(self, m_x, m_y, m_yaw, target_pos):
        dx = float(target_pos[0]) - float(m_x)
        dy = float(target_pos[1]) - float(m_y)
        cos_yaw = math.cos(m_yaw)
        sin_yaw = math.sin(m_yaw)
        local_x = cos_yaw * dx + sin_yaw * dy
        local_y = -sin_yaw * dx + cos_yaw * dy
        lookahead2 = max(local_x * local_x + local_y * local_y, 1e-6)

        steering_angle = math.atan2(2.0 * self.wheel_base * local_y, lookahead2)
        steering_angle = clamp(steering_angle, -self.config.max_steer, self.config.max_steer)

        v = clamp(self.last_v_cmd if self.last_v_cmd > 0.0 else self.last_target_speed, 0.0, self._speed_limit())
        w = v * math.tan(steering_angle) / self.wheel_base
        w = clamp(w, -self.config.max_yaw_rate, self.config.max_yaw_rate)
        return v, w

    def calc_mpc_like(self, m_x, m_y, m_yaw):
        self.m_x = float(m_x)
        self.m_y = float(m_y)
        self.m_yaw = float(m_yaw)
        return self.compute_control(1.0 / self.control_rate)

    def search_vehicle_initial_index(self):
        if self.trajectory is None:
            return
        index, _ = nearest_index(self.trajectory, self.m_x, self.m_y, start_index=None, search_window=None)
        self.vehpos_initial_index = index

    def find_nearest_point_index(self, target_x, target_y):
        if self.trajectory is None:
            return -1
        index, _ = nearest_index(self.trajectory, target_x, target_y, start_index=None, search_window=None)
        return index

    def update_vehpos_index(self):
        if self.trajectory is None:
            return
        index, distance = nearest_index(
            self.trajectory,
            self.m_x,
            self.m_y,
            start_index=self.vehpos_initial_index,
            search_window=140,
        )
        if distance > 25.0:
            self.search_vehicle_initial_index()
        else:
            self.vehpos_initial_index = index

    def search_target_pos(self):
        if self.trajectory is None:
            return
        lookahead = self.config.lookahead_base + self.config.lookahead_gain * max(self.m_v, self.last_v_cmd)
        target_s = self.trajectory.points[self.vehpos_initial_index].s + lookahead
        x, y, _, _, _ = sample_reference_at_s(self.trajectory, target_s)
        target_pos_index = self.find_nearest_point_index(x, y)
        if target_pos_index >= 0:
            self.targetPos_Info[0] = self.X_points[target_pos_index]
            self.targetPos_Info[1] = self.Y_points[target_pos_index]

    def _reference_at_s(self, s):
        if self.trajectory is None:
            self.load_route(self.route_file)
        return sample_reference_at_s(self.trajectory, s)

    def _speed_limit(self):
        return min(float(self.config.max_speed), float(self.config.hard_speed_limit), 19.0)

    def _log_debug(self, nearest, lateral, v_cmd, w_cmd):
        if self.debug_logger is None:
            return
        self.debug_logger.log(
            {
                "x": self.m_x,
                "y": self.m_y,
                "yaw": self.m_yaw,
                "speed": self.m_v,
                "index": self.vehpos_initial_index,
                "target_index": lateral.target_index,
                "target_x": lateral.target_x,
                "target_y": lateral.target_y,
                "cross_track_error": lateral.cross_track_error,
                "heading_error": lateral.heading_error,
                "curvature": nearest.curvature,
                "target_speed": self.last_target_speed,
                "v_cmd": v_cmd,
                "w_cmd": w_cmd,
                "steering_angle": lateral.steering_angle,
            }
        )


def parse_args():
    parser = argparse.ArgumentParser(description="Unity intelligent vehicle controller.")
    parser.add_argument("--route-file", default=None, help="Route JSON file. Defaults to ROUTE_FILE env or built-in route.")
    parser.add_argument("--net-config", default=None, help="vehicle_name,server_ip,udp_port,udp_send_port")
    parser.add_argument("--log-dir", default="log", help="Directory for control debug CSV logs.")
    parser.add_argument("--run-route-dir", default="run_route", help="Directory for actual trajectory JSON files.")
    parser.add_argument("--log-every-frames", type=int, default=3, help="Control debug log stride in frames.")
    parser.add_argument(
        "--route-save-every-frames",
        type=int,
        default=3,
        help="Actual trajectory JSON save stride in frames.",
    )

    control_log_group = parser.add_mutually_exclusive_group()
    control_log_group.add_argument(
        "--enable-control-log",
        dest="control_log_enabled",
        action="store_true",
        default=True,
        help="Enable control debug CSV logging.",
    )
    control_log_group.add_argument(
        "--disable-control-log",
        dest="control_log_enabled",
        action="store_false",
        help="Disable control debug CSV logging.",
    )

    trajectory_log_group = parser.add_mutually_exclusive_group()
    trajectory_log_group.add_argument(
        "--enable-trajectory-log",
        dest="trajectory_log_enabled",
        action="store_true",
        default=True,
        help="Enable actual vehicle trajectory JSON logging.",
    )
    trajectory_log_group.add_argument(
        "--disable-trajectory-log",
        dest="trajectory_log_enabled",
        action="store_false",
        help="Disable actual vehicle trajectory JSON logging.",
    )
    return parser.parse_args()


def install_shutdown_handlers(control):
    def shutdown(signum, frame):
        if control.debug_logger is not None:
            control.debug_logger.close()
        raise SystemExit(128 + int(signum))

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)


if __name__ == "__main__":
    args = parse_args()
    control = Control(
        route_file=args.route_file,
        net_config=args.net_config,
        control_log_enabled=args.control_log_enabled,
        trajectory_log_enabled=args.trajectory_log_enabled,
        log_dir=args.log_dir,
        run_route_dir=args.run_route_dir,
        log_every_n_frames=args.log_every_frames,
        route_save_every_n_frames=args.route_save_every_frames,
    )
    install_shutdown_handlers(control)
    try:
        control.udp_client.start()
        control.control_node()
    finally:
        if control.debug_logger is not None:
            control.debug_logger.close()
