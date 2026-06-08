import json
import math
from bisect import bisect_right


def clamp(value, low, high):
    return max(low, min(high, value))


def wrap_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class TrajectoryPoint:
    __slots__ = ("x", "y", "yaw", "curvature", "target_speed", "s")

    def __init__(self, x, y, yaw, curvature, target_speed, s):
        self.x = float(x)
        self.y = float(y)
        self.yaw = float(yaw)
        self.curvature = float(curvature)
        self.target_speed = float(target_speed)
        self.s = float(s)


class ReferenceTrajectory:
    __slots__ = ("points", "closed", "length")

    def __init__(self, points, closed, length):
        self.points = points
        self.closed = bool(closed)
        self.length = float(length)

    def __len__(self):
        return len(self.points)


class SpeedProfileConfig:
    def __init__(
        self,
        max_speed=18.0,
        min_speed=5.0,
        max_accel=3.5,
        max_decel=5.0,
        max_lateral_accel=4.0,
        curvature_speed_gain=55.0,
        speed_filter_alpha=0.35,
    ):
        self.max_speed = float(max_speed)
        self.min_speed = float(min_speed)
        self.max_accel = float(max_accel)
        self.max_decel = float(max_decel)
        self.max_lateral_accel = float(max_lateral_accel)
        self.curvature_speed_gain = float(curvature_speed_gain)
        self.speed_filter_alpha = float(speed_filter_alpha)


def load_route_json(file_path):
    with open(file_path, "r", encoding="utf-8") as file:
        json_track = json.load(file)

    if isinstance(json_track, list):
        xs = []
        ys = []
        for point in json_track:
            if "x" not in point or "y" not in point:
                raise ValueError("Route list points must contain x and y.")
            xs.append(float(point["x"]))
            ys.append(float(point["y"]))
    elif isinstance(json_track, dict) and "X" in json_track and "Y" in json_track:
        xs = [float(x) for x in json_track["X"]]
        ys = [float(y) for y in json_track["Y"]]
    else:
        raise ValueError(
            "Unsupported route format. Expected [{'x': ..., 'y': ...}, ...] "
            "or {'X': [...], 'Y': [...]}."
        )

    if len(xs) != len(ys) or len(xs) == 0:
        raise ValueError("Route file must contain the same non-zero number of X and Y points.")

    return sanitize_path(xs, ys)


def sanitize_path(xs, ys, min_distance=1e-6):
    clean_x = []
    clean_y = []
    for x, y in zip(xs, ys):
        if not clean_x or math.hypot(x - clean_x[-1], y - clean_y[-1]) > min_distance:
            clean_x.append(float(x))
            clean_y.append(float(y))

    if len(clean_x) < 2:
        raise ValueError("Route must contain at least two distinct points.")

    closed = math.hypot(clean_x[0] - clean_x[-1], clean_y[0] - clean_y[-1]) < 1e-3
    if closed and len(clean_x) > 2:
        clean_x.pop()
        clean_y.pop()

    return clean_x, clean_y, closed


def path_length(xs, ys, closed=False):
    if len(xs) < 2:
        return 0.0
    total = 0.0
    for i in range(1, len(xs)):
        total += math.hypot(xs[i] - xs[i - 1], ys[i] - ys[i - 1])
    if closed:
        total += math.hypot(xs[0] - xs[-1], ys[0] - ys[-1])
    return total


def resample_polyline(xs, ys, spacing=0.5, closed=False):
    if len(xs) < 2:
        return list(xs), list(ys)

    work_x = list(xs)
    work_y = list(ys)
    if closed:
        work_x.append(xs[0])
        work_y.append(ys[0])

    cumulative = [0.0]
    for i in range(1, len(work_x)):
        cumulative.append(
            cumulative[-1] + math.hypot(work_x[i] - work_x[i - 1], work_y[i] - work_y[i - 1])
        )

    total = cumulative[-1]
    if total <= 1e-9:
        return [xs[0]], [ys[0]]

    spacing = max(0.05, float(spacing))
    if closed:
        sample_count = max(3, int(round(total / spacing)))
        distances = [i * total / sample_count for i in range(sample_count)]
    else:
        sample_count = max(2, int(math.ceil(total / spacing)) + 1)
        distances = [min(i * spacing, total) for i in range(sample_count - 1)]
        distances.append(total)

    out_x = []
    out_y = []
    seg_index = 0
    for distance in distances:
        while seg_index < len(cumulative) - 2 and cumulative[seg_index + 1] < distance:
            seg_index += 1
        seg_len = cumulative[seg_index + 1] - cumulative[seg_index]
        ratio = 0.0 if seg_len <= 1e-9 else (distance - cumulative[seg_index]) / seg_len
        out_x.append(work_x[seg_index] + ratio * (work_x[seg_index + 1] - work_x[seg_index]))
        out_y.append(work_y[seg_index] + ratio * (work_y[seg_index + 1] - work_y[seg_index]))

    return out_x, out_y


def cumulative_distances(xs, ys, closed=False):
    if not xs:
        return [], 0.0
    s_values = [0.0]
    for i in range(1, len(xs)):
        s_values.append(s_values[-1] + math.hypot(xs[i] - xs[i - 1], ys[i] - ys[i - 1]))
    total = s_values[-1]
    if closed and len(xs) > 1:
        total += math.hypot(xs[0] - xs[-1], ys[0] - ys[-1])
    return s_values, total


def compute_yaw_curvature(xs, ys, closed=False):
    n = len(xs)
    if n == 0:
        return [], []
    if n == 1:
        return [0.0], [0.0]

    yaws = []
    curvatures = []
    for i in range(n):
        if closed:
            im = (i - 1) % n
            ip = (i + 1) % n
        else:
            im = max(0, i - 1)
            ip = min(n - 1, i + 1)

        dx = xs[ip] - xs[im]
        dy = ys[ip] - ys[im]
        yaw = math.atan2(dy, dx) if abs(dx) + abs(dy) > 1e-9 else 0.0
        yaws.append(yaw)

        if (not closed) and (i == 0 or i == n - 1):
            curvatures.append(0.0)
            continue

        p0 = im
        p1 = i
        p2 = ip
        h1 = math.atan2(ys[p1] - ys[p0], xs[p1] - xs[p0])
        h2 = math.atan2(ys[p2] - ys[p1], xs[p2] - xs[p1])
        d1 = math.hypot(xs[p1] - xs[p0], ys[p1] - ys[p0])
        d2 = math.hypot(xs[p2] - xs[p1], ys[p2] - ys[p1])
        ds = max(0.5 * (d1 + d2), 1e-6)
        curvatures.append(wrap_angle(h2 - h1) / ds)

    if not closed and n > 2:
        curvatures[0] = curvatures[1]
        curvatures[-1] = curvatures[-2]

    return yaws, curvatures


def _segment_distances(xs, ys, closed):
    n = len(xs)
    distances = [0.0] * n
    if n < 2:
        return distances
    for i in range(n - 1):
        distances[i] = math.hypot(xs[i + 1] - xs[i], ys[i + 1] - ys[i])
    distances[-1] = math.hypot(xs[0] - xs[-1], ys[0] - ys[-1]) if closed else distances[-2]
    return distances


def plan_speed_profile(xs, ys, curvatures, closed, config):
    n = len(curvatures)
    if n == 0:
        return []

    max_speed = min(float(config.max_speed), 19.0)
    min_speed = min(float(config.min_speed), max_speed)
    base = []
    for curvature in curvatures:
        k = abs(curvature)
        curvature_limited = math.sqrt(max(config.max_lateral_accel, 0.1) / max(k, 1e-4))
        shaped = min_speed + (max_speed - min_speed) / (1.0 + config.curvature_speed_gain * k)
        base.append(clamp(min(curvature_limited, shaped), min_speed, max_speed))

    speeds = base[:]
    ds = _segment_distances(xs, ys, closed)

    passes = 4 if closed else 1
    for _ in range(passes):
        for i in range(1, n):
            reachable = math.sqrt(max(0.0, speeds[i - 1] ** 2 + 2.0 * config.max_accel * ds[i - 1]))
            speeds[i] = min(speeds[i], reachable)
        if closed and n > 1:
            reachable = math.sqrt(max(0.0, speeds[-1] ** 2 + 2.0 * config.max_accel * ds[-1]))
            speeds[0] = min(speeds[0], reachable)

        for i in range(n - 2, -1, -1):
            reachable = math.sqrt(max(0.0, speeds[i + 1] ** 2 + 2.0 * config.max_decel * ds[i]))
            speeds[i] = min(speeds[i], reachable)
        if closed and n > 1:
            reachable = math.sqrt(max(0.0, speeds[0] ** 2 + 2.0 * config.max_decel * ds[-1]))
            speeds[-1] = min(speeds[-1], reachable)

    alpha = clamp(config.speed_filter_alpha, 0.0, 1.0)
    for _ in range(3):
        filtered = speeds[:]
        for i in range(n):
            if closed:
                prev_i = (i - 1) % n
                next_i = (i + 1) % n
            else:
                if i == 0 or i == n - 1:
                    continue
                prev_i = i - 1
                next_i = i + 1
            neighbor_mean = 0.5 * (speeds[prev_i] + speeds[next_i])
            filtered[i] = (1.0 - alpha) * speeds[i] + alpha * neighbor_mean
        speeds = [clamp(v, min_speed, max_speed) for v in filtered]

    return speeds


def build_reference_trajectory(xs, ys, closed, speed_config):
    yaws, curvatures = compute_yaw_curvature(xs, ys, closed)
    speeds = plan_speed_profile(xs, ys, curvatures, closed, speed_config)
    s_values, total_length = cumulative_distances(xs, ys, closed)
    points = []
    for i in range(len(xs)):
        points.append(TrajectoryPoint(xs[i], ys[i], yaws[i], curvatures[i], speeds[i], s_values[i]))
    return ReferenceTrajectory(points, closed, total_length)


def sample_reference_at_s(reference, s):
    points = reference.points
    if not points:
        raise ValueError("Reference trajectory is empty.")
    if len(points) == 1 or reference.length <= 1e-9:
        p = points[0]
        return p.x, p.y, p.yaw, p.curvature, p.target_speed

    if reference.closed:
        s = s % reference.length
    else:
        s = clamp(s, 0.0, reference.length)

    s_values = [p.s for p in points]
    i = max(0, bisect_right(s_values, s) - 1)
    if i >= len(points) - 1:
        if not reference.closed:
            p = points[-1]
            return p.x, p.y, p.yaw, p.curvature, p.target_speed
        j = 0
        seg_len = reference.length - points[i].s
    else:
        j = i + 1
        seg_len = points[j].s - points[i].s

    ratio = 0.0 if seg_len <= 1e-9 else (s - points[i].s) / seg_len
    if i == len(points) - 1 and reference.closed:
        ratio = 0.0 if seg_len <= 1e-9 else (s - points[i].s) / seg_len

    p0 = points[i]
    p1 = points[j]
    yaw = wrap_angle(p0.yaw + ratio * wrap_angle(p1.yaw - p0.yaw))
    x = p0.x + ratio * (p1.x - p0.x)
    y = p0.y + ratio * (p1.y - p0.y)
    curvature = p0.curvature + ratio * (p1.curvature - p0.curvature)
    target_speed = p0.target_speed + ratio * (p1.target_speed - p0.target_speed)
    return x, y, yaw, curvature, target_speed


def nearest_index(reference, x, y, start_index=0, search_window=120):
    points = reference.points
    n = len(points)
    if n == 0:
        return 0, float("inf")

    if start_index is None or start_index < 0 or search_window is None or search_window <= 0:
        candidates = range(n)
    elif reference.closed:
        window = min(n, int(search_window))
        candidates = ((start_index + i) % n for i in range(window))
    else:
        end = min(n, start_index + int(search_window))
        candidates = range(max(0, start_index), end)

    best_index = 0
    best_distance = float("inf")
    for i in candidates:
        point = points[i]
        distance = math.hypot(x - point.x, y - point.y)
        if distance < best_distance:
            best_distance = distance
            best_index = i

    return best_index, best_distance
