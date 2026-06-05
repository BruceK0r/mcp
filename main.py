import bisect
import json
import math
import os
import time
from debug_logger import RuntimeDebugLogger
from my_udp import UDPClient


def clamp(value, low, high):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return low
    if not math.isfinite(value):
        return low
    return max(low, min(high, value))


def normalize_angle(angle):
    if not math.isfinite(angle):
        return 0.0
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def rate_limit(target, previous, rise_rate, fall_rate, dt):
    if not math.isfinite(target):
        target = 0.0
    if not math.isfinite(previous):
        previous = 0.0

    delta = target - previous
    max_delta = (rise_rate if delta >= 0.0 else fall_rate) * dt
    return previous + clamp(delta, -max_delta, max_delta)


class Control:
    def __init__(self, debug_logger=None):

        self.vehicle_name = '1'
        self.udp_port = 9000
        self.udp_send_port = 9001
        self.server_ip = '192.168.1.100'
        self.debug_logger = debug_logger
        self.cycle_count = 0
        self._last_fallback_warning_time = 0.0

        net = os.environ.get("NET_CONFIG", "RST6phjHTlYisx4dqWmCnNpkYfTV,127.0.0.1,1164,1165")
        if net != "":
            net = net.split(",")
            self.vehicle_name = net[0]
            self.server_ip = net[1]
            self.udp_port = int(net[2])
            self.udp_send_port = int(net[3])

        self.vehicle_name = os.environ.get("VEHICLE_NAME", self.vehicle_name)
        self.server_ip = os.environ.get("SERVER_IP", self.server_ip)
        self.udp_port = int(os.environ.get("UDP_PORT", self.udp_port))
        self.udp_send_port = int(os.environ.get("UDP_SEND_PORT", self.udp_send_port))

        print(self.vehicle_name)
        print(self.udp_port)
        print(self.udp_send_port)
        print(self.server_ip)
        self.udp_client = UDPClient(self.server_ip, self.udp_port, self.udp_send_port, self.vehicle_name)

        self.m_v = 0.0
        self.m_x = 0.0
        self.m_y = 0.0
        self.m_yaw = 0.0

        self.vehpos_initial_index = 0
        self.target_index = -1
        self.num_preview = 8.0
        self.targetPos_Info = [0.0, 0.0]
        self.Y_points = []
        self.X_points = []
        self.path_yaws = []
        self.path_curvatures = []
        self.segment_lengths = []
        self.cumulative_lengths = []
        self.path_length = 0.0
        self.closed_path = False
        self._has_valid_index = False

        self.control_rate = 10  # hz
        self.nominal_dt = 1.0 / self.control_rate
        self.dt = self.nominal_dt
        self.min_control_dt = 0.08
        self.max_control_dt = 0.16
        self._last_loop_monotonic = None
        self.wheel_base = 2.7

        # Runtime choices. Keep python main.py unchanged; override with env vars when needed.
        self.route_file = os.environ.get("ROUTE_FILE", "exp_routes/Big.json")
        self.controller_mode = os.environ.get(
            "CONTROLLER_MODE",
            os.environ.get("CONTROL_MODE", "mpc")
        ).strip().lower()

        # Safety and smoothing limits. The competition rule forbids v >= 20 m/s.
        self.max_speed = 18.0
        self.cruise_speed = 16.0
        self.min_tracking_speed = 3.0
        self.max_steer_angle = math.radians(32.0)
        self.max_w = 1.45
        self.max_accel = 6.0
        self.max_decel = 10.0
        self.max_w_rate = 2.0
        self.max_lateral_accel = 4.5
        self.last_cmd_v = 0.0
        self.last_cmd_w = 0.0

        # Tracking parameters.
        self.lookahead_min = 5.5
        self.lookahead_max = 13.5
        self.lookahead_gain = 0.35
        self.curvature_preview_distance = 30.0
        self.curvature_sample_step = 3.0
        self.relocalize_distance = 14.0
        self.local_search_back = 25
        self.local_search_forward = 90

        # Lightweight MPC-like parameters. Horizon is short enough for 10 Hz control.
        self.mpc_horizon = 10
        self.mpc_position_weight = 0.8
        self.mpc_heading_weight = 1.3
        self.mpc_cte_weight = 6.5
        self.mpc_inside_weight = 4.5
        self.mpc_terminal_weight = 8.0
        self.mpc_speed_weight = 2.0
        self.mpc_yaw_rate_weight = 0.18
        self.mpc_delta_v_weight = 0.55
        self.mpc_delta_w_weight = 3.2

    def control_node(self):
        route_loaded = self.load_route(self.route_file)
        print("route:", self.route_file)
        print("controller:", self.controller_mode)
        self._write_debug_metadata(route_loaded)

        while True:
            start_monotonic, loop_time_ms = self._begin_control_cycle()
            self.cycle_count += 1
            vehicle_data = self.udp_client.get_vehicle_state()
            state_valid = self._update_vehicle_state(vehicle_data)
            route_ready = self._route_ready()

            if not state_valid or not route_ready:
                self._warn_fallback_reason(state_valid, route_ready)
                v, w = self._finalize_control(0.0, 0.0)
                self._log_trajectory(
                    state_valid=state_valid,
                    route_ready=route_ready,
                    v=v,
                    w=w,
                    calc_time_ms=0.0,
                    loop_time_ms=loop_time_ms,
                    status="fallback"
                )
                self.udp_client.send_control_command(v, w)
                self._sleep_until_next_cycle(start_monotonic)
                continue

            self.update_vehpos_index()
            self.search_target_pos()

            calc_start = time.perf_counter()
            if self.controller_mode in ("pure_pursuit", "pure-pursuit", "pp"):
                v, w = self.calc_pure_pursuit(self.m_x, self.m_y, self.m_yaw, self.targetPos_Info)
            else:
                v, w = self.calc_mpc_like(self.m_x, self.m_y, self.m_yaw)
            calc_time_ms = (time.perf_counter() - calc_start) * 1000.0

            v, w = self._finalize_control(v, w)
            self._log_trajectory(
                state_valid=state_valid,
                route_ready=route_ready,
                v=v,
                w=w,
                calc_time_ms=calc_time_ms,
                loop_time_ms=loop_time_ms,
                status="tracking"
            )
            self.udp_client.send_control_command(v, w)
            self._sleep_until_next_cycle(start_monotonic)

    def load_route(self, file_path):
        self._clear_route()
        try:
            with open(file_path, 'r', encoding='utf-8') as file:
                json_track = json.load(file)

            if isinstance(json_track, list):
                x_points = [point["x"] for point in json_track]
                y_points = [point["y"] for point in json_track]
            elif isinstance(json_track, dict) and "X" in json_track and "Y" in json_track:
                x_points = json_track["X"]
                y_points = json_track["Y"]
            else:
                print("Unsupported route format:", file_path)
                return False

            if len(x_points) != len(y_points):
                print("Route X/Y length mismatch:", file_path)
                return False

            points = []
            for x, y in zip(x_points, y_points):
                x = float(x)
                y = float(y)
                if math.isfinite(x) and math.isfinite(y):
                    points.append((x, y))

            if len(points) < 2:
                print("Route has fewer than two valid points:", file_path)
                return False

            first_x, first_y = points[0]
            last_x, last_y = points[-1]
            end_gap = math.hypot(last_x - first_x, last_y - first_y)
            self.closed_path = end_gap < 2.0
            if end_gap < 0.25 and len(points) > 2:
                points.pop()

            self.X_points = [p[0] for p in points]
            self.Y_points = [p[1] for p in points]
            self._prepare_path_geometry()
            self._has_valid_index = False
            return self._route_ready()
        except Exception as exc:
            print("Failed to load route:", file_path, exc)
            self._clear_route()
            return False

    def calc_pure_pursuit(self, m_x, m_y, m_yaw, target_pos):
        if not self._route_ready() or target_pos is None or len(target_pos) < 2:
            return 0.0, 0.0

        target_x = float(target_pos[0])
        target_y = float(target_pos[1])
        if not (math.isfinite(target_x) and math.isfinite(target_y)):
            return 0.0, 0.0

        dx = target_x - m_x
        dy = target_y - m_y
        lookahead = max(math.hypot(dx, dy), 0.1)
        alpha = normalize_angle(math.atan2(dy, dx) - m_yaw)

        steering_angle = math.atan2(2.0 * self.wheel_base * math.sin(alpha), lookahead)
        steering_angle = clamp(steering_angle, -self.max_steer_angle, self.max_steer_angle)

        v = self.calc_dynamic_speed(m_x, m_y, m_yaw, self.vehpos_initial_index)
        if abs(alpha) > math.radians(75.0):
            v = min(v, 5.5)

        w = v * math.tan(steering_angle) / self.wheel_base
        return v, clamp(w, -self.max_w, self.max_w)

    def calc_mpc_like(self, m_x, m_y, m_yaw):
        if not self._route_ready():
            return 0.0, 0.0

        desired_speed = self.calc_dynamic_speed(m_x, m_y, m_yaw, self.vehpos_initial_index)
        _, pure_pursuit_w = self.calc_pure_pursuit(m_x, m_y, m_yaw, self.targetPos_Info)

        v_candidates = self._build_speed_candidates(desired_speed)
        w_candidates = self._build_yaw_rate_candidates(pure_pursuit_w)
        start_s = self._s_at_index(self.vehpos_initial_index)

        best_cost = float('inf')
        best_v = desired_speed
        best_w = pure_pursuit_w

        for candidate_v in v_candidates:
            for candidate_w in w_candidates:
                x = m_x
                y = m_y
                yaw = m_yaw
                progress = 0.0
                cost = self._control_effort_cost(candidate_v, candidate_w, desired_speed)

                terminal_cte = 0.0
                for _ in range(self.mpc_horizon):
                    x += candidate_v * math.cos(yaw) * self.dt
                    y += candidate_v * math.sin(yaw) * self.dt
                    yaw = normalize_angle(yaw + candidate_w * self.dt)
                    progress += max(candidate_v, 0.0) * self.dt

                    ref_x, ref_y, ref_yaw, ref_curvature, _ = self._reference_at_s(start_s + progress)
                    longitudinal_error, cross_track_error = self._reference_frame_errors(
                        x,
                        y,
                        ref_x,
                        ref_y,
                        ref_yaw
                    )
                    heading_error = normalize_angle(yaw - ref_yaw)
                    inside_error = self._inside_cut_error(cross_track_error, ref_curvature)
                    curve_gain = self._curve_cost_gain(ref_curvature)

                    cost += self.mpc_position_weight * longitudinal_error * longitudinal_error
                    cost += self.mpc_cte_weight * cross_track_error * cross_track_error
                    cost += self.mpc_inside_weight * curve_gain * inside_error * inside_error
                    cost += self.mpc_heading_weight * heading_error * heading_error
                    terminal_cte = cross_track_error

                cost += self.mpc_terminal_weight * terminal_cte * terminal_cte

                if cost < best_cost:
                    best_cost = cost
                    best_v = candidate_v
                    best_w = candidate_w

        return best_v, best_w

    def calc_dynamic_speed(self, m_x, m_y, m_yaw, nearest_index):
        if not self._route_ready():
            return 0.0

        nearest_index = self._valid_index(nearest_index)
        ref_x, ref_y, ref_yaw, ref_curvature, _ = self._reference_at_s(self._s_at_index(nearest_index))
        _, signed_cte = self._reference_frame_errors(m_x, m_y, ref_x, ref_y, ref_yaw)
        cross_track_error = abs(signed_cte)
        heading_error = abs(normalize_angle(m_yaw - ref_yaw))
        max_curvature = max(
            abs(ref_curvature),
            self._max_abs_curvature_ahead(nearest_index, self.curvature_preview_distance)
        )

        if max_curvature < 1e-4:
            curve_speed = self.max_speed
        else:
            curve_speed = math.sqrt(self.max_lateral_accel / max_curvature)

        speed = min(self.cruise_speed, curve_speed, self.max_speed)
        cte_factor = clamp(1.0 - 0.10 * cross_track_error, 0.35, 1.0)
        heading_factor = clamp(1.0 - 0.60 * heading_error, 0.40, 1.0)
        speed *= cte_factor * heading_factor

        if cross_track_error > 0.6:
            cte_speed_cap = clamp(
                self.cruise_speed - 1.3 * (cross_track_error - 0.6),
                6.0,
                self.cruise_speed
            )
            speed = min(speed, cte_speed_cap)

        if max_curvature > 0.01 and cross_track_error > 1.0:
            speed = min(speed, max(self.min_tracking_speed, curve_speed * 0.85))

        if cross_track_error > 3.0 or heading_error > math.radians(45.0):
            speed = min(speed, 7.0)

        if cross_track_error > 5.0 or heading_error > math.radians(60.0):
            speed = min(speed, 5.5)

        return clamp(speed, self.min_tracking_speed, self.max_speed)

    def search_vehicle_initial_index(self):
        nearest_index = self.find_nearest_point_index(self.m_x, self.m_y)
        if nearest_index >= 0:
            self.vehpos_initial_index = nearest_index
            self._has_valid_index = True

    def find_nearest_point_index(self, target_x, target_y):
        if not self._route_ready():
            return -1

        min_distance = float('inf')
        nearest_index = -1

        for i in range(len(self.X_points)):
            distance = math.hypot(target_x - self.X_points[i], target_y - self.Y_points[i])
            if distance < min_distance:
                min_distance = distance
                nearest_index = i

        return nearest_index

    def update_vehpos_index(self):
        if not self._route_ready():
            self.vehpos_initial_index = 0
            self._has_valid_index = False
            return

        if not self._has_valid_index:
            self.search_vehicle_initial_index()
            return

        nearest_index, min_distance = self._nearest_index_local(
            self.m_x,
            self.m_y,
            self.vehpos_initial_index,
            self.local_search_back,
            self.local_search_forward
        )

        if min_distance > self.relocalize_distance:
            self.search_vehicle_initial_index()
        elif nearest_index >= 0:
            self.vehpos_initial_index = nearest_index

    def search_target_pos(self):
        if not self._route_ready():
            self.targetPos_Info[0] = self.m_x
            self.targetPos_Info[1] = self.m_y
            self.target_index = -1
            return

        lookahead = self._lookahead_distance()
        target_x, target_y, _, _, target_index = self._reference_from_index_distance(
            self.vehpos_initial_index,
            lookahead
        )
        self.targetPos_Info[0] = target_x
        self.targetPos_Info[1] = target_y
        self.target_index = target_index

    def _build_speed_candidates(self, desired_speed):
        desired_speed = clamp(desired_speed, 0.0, self.max_speed)
        lower = max(0.0, self.last_cmd_v - self.max_decel * self.dt)
        rate_upper = min(self.max_speed, self.last_cmd_v + self.max_accel * self.dt)
        if desired_speed >= self.last_cmd_v:
            upper = min(rate_upper, desired_speed * 1.03 + 0.15)
        else:
            upper = min(rate_upper, self.last_cmd_v)
        upper = max(lower, upper)
        raw_values = [
            lower,
            upper,
            self.last_cmd_v,
            desired_speed * 0.70,
            desired_speed * 0.85,
            desired_speed,
            desired_speed * 1.03,
            self.m_v,
        ]
        return self._unique_sorted(clamp(v, lower, upper) for v in raw_values)

    def _build_yaw_rate_candidates(self, pure_pursuit_w):
        if not math.isfinite(pure_pursuit_w):
            pure_pursuit_w = self.last_cmd_w
        pure_pursuit_w = clamp(pure_pursuit_w, -self.max_w, self.max_w)

        lower = max(-self.max_w, self.last_cmd_w - self.max_w_rate * self.dt)
        upper = min(self.max_w, self.last_cmd_w + self.max_w_rate * self.dt)
        center = clamp(0.65 * self.last_cmd_w + 0.35 * pure_pursuit_w, lower, upper)
        step = clamp(0.45 * self.max_w_rate * max(self.dt, self.min_control_dt), 0.04, 0.10)
        raw_values = [lower, upper, self.last_cmd_w, pure_pursuit_w, center]
        for offset in range(-4, 5):
            raw_values.append(center + offset * step)

        if lower <= 0.0 <= upper and abs(center) < 0.12 and abs(pure_pursuit_w) < 0.20:
            raw_values.append(0.0)
        return self._unique_sorted(clamp(w, lower, upper) for w in raw_values)

    def _control_effort_cost(self, candidate_v, candidate_w, desired_speed):
        norm_v = candidate_v / max(self.max_speed, 0.1)
        norm_w = candidate_w / max(self.max_w, 0.1)
        norm_delta_v = (candidate_v - self.last_cmd_v) / max(self.max_speed, 0.1)
        norm_delta_w = (candidate_w - self.last_cmd_w) / max(self.max_w, 0.1)
        norm_speed_error = (candidate_v - desired_speed) / max(self.max_speed, 0.1)

        return (
            0.04 * norm_v * norm_v
            + self.mpc_yaw_rate_weight * norm_w * norm_w
            + self.mpc_delta_v_weight * norm_delta_v * norm_delta_v
            + self.mpc_delta_w_weight * norm_delta_w * norm_delta_w
            + self.mpc_speed_weight * norm_speed_error * norm_speed_error
        )

    def _finalize_control(self, v, w):
        if not math.isfinite(v):
            v = 0.0
        if not math.isfinite(w):
            w = 0.0

        w = clamp(w, -self.max_w, self.max_w)
        v = clamp(v, 0.0, self.max_speed)

        if abs(w) > 0.05:
            v = min(v, max(self.min_tracking_speed, self.max_lateral_accel / abs(w)))

        v = rate_limit(v, self.last_cmd_v, self.max_accel, self.max_decel, self.dt)
        w = rate_limit(w, self.last_cmd_w, self.max_w_rate, self.max_w_rate, self.dt)
        v = clamp(v, 0.0, self.max_speed)
        w = clamp(w, -self.max_w, self.max_w)

        if abs(w) > 0.05:
            v = min(v, max(self.min_tracking_speed, self.max_lateral_accel / abs(w)))

        if abs(v) < 1e-3:
            v = 0.0
        if abs(w) < 1e-4:
            w = 0.0

        self.last_cmd_v = v
        self.last_cmd_w = w
        return v, w

    def _update_vehicle_state(self, vehicle_data):
        try:
            if vehicle_data is None or vehicle_data.name == "":
                return False

            x = float(vehicle_data.x)
            y = float(vehicle_data.y)
            yaw_deg = float(vehicle_data.yaw)

            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(yaw_deg)):
                return False

            self.m_x = x
            self.m_y = y
            self.m_yaw = normalize_angle(math.radians(yaw_deg))

            try:
                speed = float(vehicle_data.speed)
            except (TypeError, ValueError):
                speed = self.last_cmd_v
            if not math.isfinite(speed):
                self.m_v = self.last_cmd_v
            else:
                self.m_v = clamp(abs(speed), 0.0, self.max_speed)
            return True
        except Exception:
            return False

    def _begin_control_cycle(self):
        now = time.monotonic()
        if self._last_loop_monotonic is None:
            self._last_loop_monotonic = now
            self.dt = self.nominal_dt
            return now, self.nominal_dt * 1000.0

        raw_dt = now - self._last_loop_monotonic
        self._last_loop_monotonic = now
        if math.isfinite(raw_dt) and raw_dt > 0.0:
            self.dt = clamp(raw_dt, self.min_control_dt, self.max_control_dt)
            loop_time_ms = raw_dt * 1000.0
        else:
            self.dt = self.nominal_dt
            loop_time_ms = self.nominal_dt * 1000.0
        return now, loop_time_ms

    def _sleep_until_next_cycle(self, start_monotonic):
        elapsed_time = time.monotonic() - start_monotonic
        sleep_time = max(self.nominal_dt - elapsed_time, 0.0)
        time.sleep(sleep_time)

    def _write_debug_metadata(self, route_loaded):
        if self.debug_logger is None:
            return
        self.debug_logger.write_metadata({
            "vehicle_name": self.vehicle_name,
            "server_ip": self.server_ip,
            "udp_port": self.udp_port,
            "udp_send_port": self.udp_send_port,
            "route_file": self.route_file,
            "route_loaded": route_loaded,
            "controller": self.controller_mode,
            "control_rate_hz": self.control_rate,
            "dt_min": self.min_control_dt,
            "dt_max": self.max_control_dt,
            "max_speed": self.max_speed,
            "cruise_speed": self.cruise_speed,
            "max_w": self.max_w,
            "max_accel": self.max_accel,
            "max_decel": self.max_decel,
            "max_w_rate": self.max_w_rate,
            "lookahead_min": self.lookahead_min,
            "lookahead_max": self.lookahead_max,
            "lookahead_gain": self.lookahead_gain,
            "mpc_horizon": self.mpc_horizon,
            "mpc_position_weight": self.mpc_position_weight,
            "mpc_heading_weight": self.mpc_heading_weight,
            "mpc_cte_weight": self.mpc_cte_weight,
            "mpc_inside_weight": self.mpc_inside_weight,
            "mpc_terminal_weight": self.mpc_terminal_weight,
            "mpc_yaw_rate_weight": self.mpc_yaw_rate_weight,
            "mpc_delta_w_weight": self.mpc_delta_w_weight,
        })

    def _warn_fallback_reason(self, state_valid, route_ready):
        now = time.time()
        if now - self._last_fallback_warning_time < 2.0:
            return
        self._last_fallback_warning_time = now

        if not route_ready:
            print("fallback stop: route is not ready. route_file =", self.route_file)
            return

        if state_valid:
            return

        debug_status = {}
        if hasattr(self.udp_client, "get_debug_status"):
            debug_status = self.udp_client.get_debug_status()

        packet_count = debug_status.get("packet_count", 0)
        last_seen_names = debug_status.get("last_seen_vehicle_names", [])

        if packet_count == 0:
            print(
                "fallback stop: no UDP vehicle state received yet.",
                "listen_port =", self.udp_port,
                "vehicle_name =", self.vehicle_name,
                "check Unity is running and sending to this port."
            )
        else:
            print(
                "fallback stop: UDP packets received, but target vehicle name was not found.",
                "vehicle_name =", self.vehicle_name,
                "seen_names =", last_seen_names,
                "fix NET_CONFIG or VEHICLE_NAME."
            )

    def _log_trajectory(self, state_valid, route_ready, v, w, calc_time_ms, loop_time_ms, status):
        if self.debug_logger is None:
            return

        target_x = self.targetPos_Info[0] if len(self.targetPos_Info) > 0 else ""
        target_y = self.targetPos_Info[1] if len(self.targetPos_Info) > 1 else ""
        cross_track_error = ""
        heading_error = ""
        path_curvature = ""

        if route_ready and self._has_valid_index and 0 <= self.vehpos_initial_index < len(self.path_yaws):
            nearest_index = self._valid_index(self.vehpos_initial_index)
            cross_track_error = self._cross_track_error(self.m_x, self.m_y, nearest_index)
            heading_error = normalize_angle(self.m_yaw - self.path_yaws[nearest_index])
            path_curvature = self.path_curvatures[nearest_index]

        self.debug_logger.log_trajectory(
            cycle=self.cycle_count,
            vehicle_name=self.vehicle_name,
            route_file=self.route_file,
            controller=self.controller_mode,
            state_valid=state_valid,
            route_ready=route_ready,
            status=status,
            x=self.m_x,
            y=self.m_y,
            yaw_rad=self.m_yaw,
            yaw_deg=math.degrees(self.m_yaw),
            speed_mps=self.m_v,
            cmd_v=v,
            cmd_w=w,
            nearest_index=self.vehpos_initial_index if route_ready else "",
            target_index=self.target_index if route_ready else "",
            target_x=target_x if route_ready else "",
            target_y=target_y if route_ready else "",
            cross_track_error=cross_track_error,
            heading_error_rad=heading_error,
            path_curvature=path_curvature,
            calc_time_ms=calc_time_ms,
            loop_time_ms=loop_time_ms,
        )

    def _clear_route(self):
        self.X_points = []
        self.Y_points = []
        self.path_yaws = []
        self.path_curvatures = []
        self.segment_lengths = []
        self.cumulative_lengths = []
        self.path_length = 0.0
        self.closed_path = False
        self.vehpos_initial_index = 0
        self.target_index = -1
        self._has_valid_index = False

    def _prepare_path_geometry(self):
        n = len(self.X_points)
        self.segment_lengths = []
        if n < 2:
            return

        last_segment_index = n if self.closed_path else n - 1
        for i in range(last_segment_index):
            j = (i + 1) % n
            self.segment_lengths.append(
                math.hypot(self.X_points[j] - self.X_points[i], self.Y_points[j] - self.Y_points[i])
            )

        self.cumulative_lengths = [0.0]
        for length in self.segment_lengths:
            self.cumulative_lengths.append(self.cumulative_lengths[-1] + max(length, 0.0))
        self.path_length = self.cumulative_lengths[-1]

        self.path_yaws = [self._estimate_path_yaw(i) for i in range(n)]
        raw_curvatures = [self._estimate_path_curvature(i) for i in range(n)]
        self.path_curvatures = self._smooth_curvatures(raw_curvatures)

    def _estimate_path_yaw(self, index):
        n = len(self.X_points)
        if n < 2:
            return 0.0

        if self.closed_path:
            prev_index = (index - 1) % n
            next_index = (index + 1) % n
        elif index == 0:
            prev_index = index
            next_index = 1
        elif index == n - 1:
            prev_index = n - 2
            next_index = index
        else:
            prev_index = index - 1
            next_index = index + 1

        dx = self.X_points[next_index] - self.X_points[prev_index]
        dy = self.Y_points[next_index] - self.Y_points[prev_index]
        if abs(dx) < 1e-9 and abs(dy) < 1e-9:
            return 0.0
        return math.atan2(dy, dx)

    def _estimate_path_curvature(self, index):
        n = len(self.X_points)
        if n < 3:
            return 0.0

        if self.closed_path:
            i0 = (index - 1) % n
            i1 = index
            i2 = (index + 1) % n
        elif index == 0 or index == n - 1:
            return 0.0
        else:
            i0 = index - 1
            i1 = index
            i2 = index + 1

        x0, y0 = self.X_points[i0], self.Y_points[i0]
        x1, y1 = self.X_points[i1], self.Y_points[i1]
        x2, y2 = self.X_points[i2], self.Y_points[i2]

        a = math.hypot(x1 - x0, y1 - y0)
        b = math.hypot(x2 - x1, y2 - y1)
        c = math.hypot(x2 - x0, y2 - y0)
        if a < 1e-6 or b < 1e-6 or c < 1e-6:
            return 0.0

        twice_area = (x1 - x0) * (y2 - y0) - (y1 - y0) * (x2 - x0)
        return 2.0 * twice_area / (a * b * c)

    def _smooth_curvatures(self, curvatures):
        if not curvatures:
            return []

        smoothed = []
        n = len(curvatures)
        for i in range(n):
            values = []
            for offset in (-2, -1, 0, 1, 2):
                j = i + offset
                if self.closed_path:
                    j %= n
                elif j < 0 or j >= n:
                    continue
                values.append(curvatures[j])
            smoothed.append(sum(values) / max(len(values), 1))
        return smoothed

    def _route_ready(self):
        return len(self.X_points) >= 2 and len(self.Y_points) == len(self.X_points) and self.path_length > 1e-6

    def _valid_index(self, index):
        if not self.X_points:
            return 0
        return int(index) % len(self.X_points) if self.closed_path else int(clamp(index, 0, len(self.X_points) - 1))

    def _nearest_index_local(self, x, y, center_index, back_count, forward_count):
        n = len(self.X_points)
        if n == 0:
            return -1, float('inf')

        min_distance = float('inf')
        nearest_index = -1

        if self.closed_path:
            offsets = range(-back_count, forward_count + 1)
            for offset in offsets:
                index = (center_index + offset) % n
                distance = math.hypot(x - self.X_points[index], y - self.Y_points[index])
                if distance < min_distance:
                    min_distance = distance
                    nearest_index = index
        else:
            start = max(0, center_index - back_count)
            end = min(n - 1, center_index + forward_count)
            for index in range(start, end + 1):
                distance = math.hypot(x - self.X_points[index], y - self.Y_points[index])
                if distance < min_distance:
                    min_distance = distance
                    nearest_index = index

        return nearest_index, min_distance

    def _lookahead_distance(self):
        speed = max(self.m_v, self.last_cmd_v)
        return clamp(self.num_preview + self.lookahead_gain * speed, self.lookahead_min, self.lookahead_max)

    def _s_at_index(self, index):
        if not self._route_ready():
            return 0.0
        index = self._valid_index(index)
        return self.cumulative_lengths[min(index, len(self.cumulative_lengths) - 1)]

    def _reference_from_index_distance(self, index, distance_ahead):
        return self._reference_at_s(self._s_at_index(index) + max(distance_ahead, 0.0))

    def _reference_at_s(self, s):
        if not self._route_ready():
            return self.m_x, self.m_y, self.m_yaw, 0.0, -1

        if self.closed_path:
            s = s % self.path_length
        else:
            s = clamp(s, 0.0, self.path_length)

        segment_count = len(self.segment_lengths)
        if segment_count <= 0 or len(self.cumulative_lengths) < 2:
            return self.m_x, self.m_y, self.m_yaw, 0.0, -1

        segment_index = bisect.bisect_right(self.cumulative_lengths, s) - 1
        segment_index = int(clamp(segment_index, 0, segment_count - 1))
        next_index = (segment_index + 1) % len(self.X_points) if self.closed_path else min(segment_index + 1, len(self.X_points) - 1)

        segment_start = self.cumulative_lengths[segment_index]
        segment_length = max(self.segment_lengths[segment_index], 1e-6)
        ratio = clamp((s - segment_start) / segment_length, 0.0, 1.0)

        x0 = self.X_points[segment_index]
        y0 = self.Y_points[segment_index]
        x1 = self.X_points[next_index]
        y1 = self.Y_points[next_index]
        ref_x = x0 + ratio * (x1 - x0)
        ref_y = y0 + ratio * (y1 - y0)
        yaw0 = self.path_yaws[segment_index] if segment_index < len(self.path_yaws) else math.atan2(y1 - y0, x1 - x0)
        yaw1 = self.path_yaws[next_index] if next_index < len(self.path_yaws) else yaw0
        ref_yaw = normalize_angle(yaw0 + ratio * normalize_angle(yaw1 - yaw0))
        if not math.isfinite(ref_yaw):
            ref_yaw = yaw0 if math.isfinite(yaw0) else 0.0

        curvature0 = self.path_curvatures[segment_index] if segment_index < len(self.path_curvatures) else 0.0
        curvature1 = self.path_curvatures[next_index] if next_index < len(self.path_curvatures) else curvature0
        ref_curvature = curvature0 + ratio * (curvature1 - curvature0)
        if not math.isfinite(ref_curvature):
            ref_curvature = 0.0
        return ref_x, ref_y, ref_yaw, ref_curvature, segment_index

    def _cross_track_error(self, x, y, nearest_index):
        if not self._route_ready():
            return 0.0
        nearest_index = self._valid_index(nearest_index)
        path_yaw = self.path_yaws[nearest_index]
        dx = x - self.X_points[nearest_index]
        dy = y - self.Y_points[nearest_index]
        return -math.sin(path_yaw) * dx + math.cos(path_yaw) * dy

    def _reference_frame_errors(self, x, y, ref_x, ref_y, ref_yaw):
        values = (x, y, ref_x, ref_y, ref_yaw)
        if not all(math.isfinite(value) for value in values):
            return 0.0, 0.0

        dx = x - ref_x
        dy = y - ref_y
        cos_yaw = math.cos(ref_yaw)
        sin_yaw = math.sin(ref_yaw)
        longitudinal_error = cos_yaw * dx + sin_yaw * dy
        cross_track_error = -sin_yaw * dx + cos_yaw * dy
        return longitudinal_error, cross_track_error

    def _inside_cut_error(self, cross_track_error, curvature):
        if not (math.isfinite(cross_track_error) and math.isfinite(curvature)):
            return 0.0
        if abs(curvature) < 1e-4:
            return 0.0
        inside_error = cross_track_error if curvature > 0.0 else -cross_track_error
        return max(inside_error, 0.0)

    def _curve_cost_gain(self, curvature):
        if not math.isfinite(curvature):
            return 0.0
        return clamp(abs(curvature) / 0.03, 0.0, 1.0)

    def _max_abs_curvature_ahead(self, start_index, distance):
        if not self._route_ready():
            return 0.0

        start_s = self._s_at_index(start_index)
        samples = max(1, int(distance / max(self.curvature_sample_step, 0.5)))
        max_curvature = 0.0
        for i in range(samples + 1):
            _, _, _, curvature, _ = self._reference_at_s(start_s + i * self.curvature_sample_step)
            max_curvature = max(max_curvature, abs(curvature))
        return max_curvature

    def _unique_sorted(self, values):
        unique = []
        for value in sorted(values):
            if not unique or abs(value - unique[-1]) > 1e-3:
                unique.append(value)
        return unique


if __name__ == '__main__':
    debug_logger = RuntimeDebugLogger().start()
    control = Control(debug_logger=debug_logger)
    try:
        control.udp_client.start()
        control.control_node()
    finally:
        debug_logger.close()
