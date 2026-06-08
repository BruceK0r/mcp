import math

from path_utils import clamp, wrap_angle


class ControllerConfig:
    def __init__(self):
        self.wheel_base = 2.7
        self.control_rate = 25.0

        self.max_speed = 18.0
        self.hard_speed_limit = 19.0
        self.min_speed = 5.0
        self.max_accel = 3.5
        self.max_decel = 5.0
        self.max_lateral_accel = 4.0
        self.curvature_speed_gain = 55.0
        self.speed_filter_alpha = 0.35

        self.max_steer = math.radians(28.0)
        self.max_steer_rate = math.radians(55.0)
        self.max_yaw_rate = 0.72

        self.lookahead_base = 1.8
        self.lookahead_gain = 0.60
        self.lookahead_min = 2.8
        self.lookahead_max = 13.5
        self.curvature_preview_distance = 8.0
        self.curvature_lookahead_gain = 4.0
        self.curvature_lookahead_min_factor = 0.60
        self.curvature_feedforward_gain = 0.35
        self.curvature_feedforward_limit = math.radians(8.0)

        self.pid_kp = 0.42
        self.pid_ki = 0.04
        self.pid_kd = 0.02
        self.pid_integral_limit = 20.0

        self.resample_spacing = 0.5
        self.smoothing_iterations = 80
        self.smoothing_data_weight = 0.08
        self.smoothing_smooth_weight = 0.28
        self.smoothing_curvature_weight = 0.04
        self.smoothing_max_step = 0.12


class PIDSpeedController:
    def __init__(self, config):
        self.config = config
        self.integral = 0.0
        self.prev_error = 0.0
        self.last_output = 0.0
        self.initialized = False

    def reset(self, current_speed=0.0):
        self.integral = 0.0
        self.prev_error = 0.0
        self.last_output = clamp(float(current_speed), 0.0, self.config.max_speed)
        self.initialized = True

    def update(self, target_speed, current_speed, dt):
        dt = max(float(dt), 1e-3)
        target_speed = clamp(float(target_speed), 0.0, self.config.max_speed)
        current_speed = max(0.0, float(current_speed))
        if not self.initialized:
            self.reset(current_speed)

        error = target_speed - current_speed
        self.integral += error * dt
        self.integral = clamp(
            self.integral,
            -self.config.pid_integral_limit,
            self.config.pid_integral_limit,
        )
        derivative = (error - self.prev_error) / dt
        correction = (
            self.config.pid_kp * error
            + self.config.pid_ki * self.integral
            + self.config.pid_kd * derivative
        )
        raw = target_speed + correction
        raw = clamp(raw, 0.0, self.config.max_speed)

        if raw >= self.last_output:
            max_delta = self.config.max_accel * dt
        else:
            max_delta = self.config.max_decel * dt
        output = clamp(raw, self.last_output - max_delta, self.last_output + max_delta)
        output = clamp(output, 0.0, self.config.max_speed)

        self.prev_error = error
        self.last_output = output
        return output


class LateralControlResult:
    __slots__ = (
        "steering_angle",
        "w_cmd",
        "target_index",
        "lookahead",
        "cross_track_error",
        "heading_error",
        "target_x",
        "target_y",
    )

    def __init__(
        self,
        steering_angle,
        w_cmd,
        target_index,
        lookahead,
        cross_track_error,
        heading_error,
        target_x,
        target_y,
    ):
        self.steering_angle = steering_angle
        self.w_cmd = w_cmd
        self.target_index = target_index
        self.lookahead = lookahead
        self.cross_track_error = cross_track_error
        self.heading_error = heading_error
        self.target_x = target_x
        self.target_y = target_y


class PurePursuitController:
    def __init__(self, config):
        self.config = config
        self.last_steer = 0.0

    def reset(self):
        self.last_steer = 0.0

    def compute(self, reference, nearest_idx, x, y, yaw, current_speed, v_cmd, dt):
        points = reference.points
        n = len(points)
        if n == 0:
            return LateralControlResult(0.0, 0.0, 0, 0.0, 0.0, 0.0, x, y)

        preview_curvature = self._preview_curvature(reference, nearest_idx, self.config.curvature_preview_distance)
        lookahead = self._lookahead(current_speed, preview_curvature)
        target_idx = self._advance_index(reference, nearest_idx, lookahead)
        target_idx = self._avoid_rear_target(reference, target_idx, x, y, yaw)
        target = points[target_idx]

        dx = target.x - x
        dy = target.y - y
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        local_x = cos_yaw * dx + sin_yaw * dy
        local_y = -sin_yaw * dx + cos_yaw * dy
        ld2 = max(local_x * local_x + local_y * local_y, 1e-6)

        steering = math.atan2(2.0 * self.config.wheel_base * local_y, ld2)
        steering += clamp(
            self.config.curvature_feedforward_gain
            * math.atan(self.config.wheel_base * preview_curvature),
            -self.config.curvature_feedforward_limit,
            self.config.curvature_feedforward_limit,
        )
        steering = clamp(steering, -self.config.max_steer, self.config.max_steer)

        max_delta = self.config.max_steer_rate * max(dt, 1e-3)
        steering = clamp(steering, self.last_steer - max_delta, self.last_steer + max_delta)
        steering = clamp(steering, -self.config.max_steer, self.config.max_steer)

        w_cmd = v_cmd * math.tan(steering) / max(self.config.wheel_base, 1e-6)
        w_cmd = clamp(w_cmd, -self.config.max_yaw_rate, self.config.max_yaw_rate)
        if abs(v_cmd) > 0.2:
            steering = math.atan2(w_cmd * self.config.wheel_base, v_cmd)
        else:
            w_cmd = 0.0
            steering = 0.0
        self.last_steer = steering

        nearest = points[nearest_idx]
        error_dx = x - nearest.x
        error_dy = y - nearest.y
        cross_track = -math.sin(nearest.yaw) * error_dx + math.cos(nearest.yaw) * error_dy
        heading_error = wrap_angle(yaw - nearest.yaw)

        return LateralControlResult(
            steering,
            w_cmd,
            target_idx,
            lookahead,
            cross_track,
            heading_error,
            target.x,
            target.y,
        )

    def _lookahead(self, speed, preview_curvature=0.0):
        speed_lookahead = clamp(
            self.config.lookahead_base + self.config.lookahead_gain * abs(float(speed)),
            self.config.lookahead_min,
            self.config.lookahead_max,
        )
        curvature_factor = 1.0 / (
            1.0 + self.config.curvature_lookahead_gain * abs(float(preview_curvature))
        )
        curvature_factor = clamp(curvature_factor, self.config.curvature_lookahead_min_factor, 1.0)
        return clamp(
            speed_lookahead * curvature_factor,
            self.config.lookahead_min,
            self.config.lookahead_max,
        )

    def _preview_curvature(self, reference, start_idx, distance):
        points = reference.points
        n = len(points)
        if n == 0:
            return 0.0

        idx = max(0, min(n - 1, int(start_idx)))
        best_curvature = points[idx].curvature
        traveled = 0.0
        for _ in range(n):
            next_idx = (idx + 1) % n if reference.closed else min(idx + 1, n - 1)
            if next_idx == idx:
                break
            traveled += math.hypot(points[next_idx].x - points[idx].x, points[next_idx].y - points[idx].y)
            idx = next_idx
            if abs(points[idx].curvature) > abs(best_curvature):
                best_curvature = points[idx].curvature
            if traveled >= distance:
                break
        return best_curvature

    def _advance_index(self, reference, start_idx, distance):
        points = reference.points
        n = len(points)
        if n <= 1:
            return 0
        idx = max(0, min(n - 1, int(start_idx)))
        traveled = 0.0
        for _ in range(n):
            next_idx = (idx + 1) % n if reference.closed else min(idx + 1, n - 1)
            if next_idx == idx:
                return idx
            traveled += math.hypot(points[next_idx].x - points[idx].x, points[next_idx].y - points[idx].y)
            idx = next_idx
            if traveled >= distance:
                return idx
        return idx

    def _avoid_rear_target(self, reference, target_idx, x, y, yaw):
        points = reference.points
        n = len(points)
        if n <= 1:
            return target_idx

        idx = target_idx
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        for _ in range(min(n, 60)):
            point = points[idx]
            local_x = cos_yaw * (point.x - x) + sin_yaw * (point.y - y)
            if local_x > 0.4:
                return idx
            next_idx = (idx + 1) % n if reference.closed else min(idx + 1, n - 1)
            if next_idx == idx:
                return idx
            idx = next_idx
        return target_idx
