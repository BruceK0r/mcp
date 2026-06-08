import heapq
import math

from path_utils import wrap_angle


class HybridNode:
    __slots__ = ("x", "y", "yaw", "cost", "parent", "steer", "direction")

    def __init__(self, x, y, yaw, cost, parent, steer, direction):
        self.x = float(x)
        self.y = float(y)
        self.yaw = float(yaw)
        self.cost = float(cost)
        self.parent = parent
        self.steer = float(steer)
        self.direction = int(direction)


class HybridAStarPlanner:
    """Low-frequency Hybrid A* interface with JSON-route fallback."""

    def __init__(
        self,
        grid_resolution=1.0,
        yaw_resolution=math.radians(15.0),
        step_size=1.5,
        wheel_base=2.7,
        max_steer=math.radians(32.0),
        steer_samples=5,
        allow_reverse=False,
        max_iterations=5000,
    ):
        self.grid_resolution = float(grid_resolution)
        self.yaw_resolution = float(yaw_resolution)
        self.step_size = float(step_size)
        self.wheel_base = float(wheel_base)
        self.max_steer = float(max_steer)
        self.steer_samples = max(3, int(steer_samples))
        self.allow_reverse = bool(allow_reverse)
        self.max_iterations = int(max_iterations)
        self.steer_cost = 0.15
        self.steer_change_cost = 0.35
        self.reverse_penalty = 3.0

    def plan(self, start=None, goal=None, grid_map=None, fallback_path=None):
        if grid_map is None or start is None or goal is None:
            return self._fallback(start, goal, fallback_path)

        path = self._search(start, goal, grid_map)
        if path:
            return path
        return self._fallback(start, goal, fallback_path)

    def _fallback(self, start, goal, fallback_path):
        if fallback_path:
            return [(float(x), float(y)) for x, y in fallback_path]
        if start is None or goal is None:
            raise ValueError("Hybrid A* needs a fallback path when map/start/goal are unavailable.")
        sx, sy = float(start[0]), float(start[1])
        gx, gy = float(goal[0]), float(goal[1])
        points = []
        for i in range(21):
            ratio = i / 20.0
            points.append((sx + ratio * (gx - sx), sy + ratio * (gy - sy)))
        return points

    def _search(self, start, goal, grid_map):
        sx, sy, syaw = float(start[0]), float(start[1]), float(start[2])
        gx, gy, gyaw = float(goal[0]), float(goal[1]), float(goal[2])

        start_node = HybridNode(sx, sy, syaw, 0.0, None, 0.0, 1)
        nodes = [start_node]
        open_heap = [(self._heuristic(sx, sy, gx, gy), 0)]
        best_cost = {self._state_key(sx, sy, syaw): 0.0}

        for _ in range(self.max_iterations):
            if not open_heap:
                break
            _, node_id = heapq.heappop(open_heap)
            node = nodes[node_id]

            if self._goal_reached(node, gx, gy, gyaw):
                return self._reconstruct(nodes, node_id, (gx, gy))

            for steer in self._steer_set():
                directions = (-1, 1) if self.allow_reverse else (1,)
                for direction in directions:
                    child = self._expand(node, node_id, steer, direction, grid_map)
                    if child is None:
                        continue
                    key = self._state_key(child.x, child.y, child.yaw)
                    if key in best_cost and best_cost[key] <= child.cost:
                        continue
                    best_cost[key] = child.cost
                    nodes.append(child)
                    child_id = len(nodes) - 1
                    priority = child.cost + self._heuristic(child.x, child.y, gx, gy)
                    heapq.heappush(open_heap, (priority, child_id))

        return []

    def _steer_set(self):
        if self.steer_samples <= 1:
            return [0.0]
        values = []
        for i in range(self.steer_samples):
            ratio = i / float(self.steer_samples - 1)
            values.append(-self.max_steer + 2.0 * self.max_steer * ratio)
        return values

    def _expand(self, node, parent_id, steer, direction, grid_map):
        travel = direction * self.step_size
        yaw_rate = math.tan(steer) / max(self.wheel_base, 1e-6)
        new_yaw = wrap_angle(node.yaw + travel * yaw_rate)
        avg_yaw = wrap_angle(0.5 * (node.yaw + new_yaw))
        new_x = node.x + travel * math.cos(avg_yaw)
        new_y = node.y + travel * math.sin(avg_yaw)

        if not self._motion_collision_free(node.x, node.y, node.yaw, steer, direction, grid_map):
            return None

        move_cost = abs(self.step_size)
        move_cost += self.steer_cost * abs(steer)
        move_cost += self.steer_change_cost * abs(steer - node.steer)
        if direction < 0:
            move_cost += self.reverse_penalty * abs(self.step_size)
        return HybridNode(new_x, new_y, new_yaw, node.cost + move_cost, parent_id, steer, direction)

    def _motion_collision_free(self, x, y, yaw, steer, direction, grid_map):
        steps = 5
        for i in range(1, steps + 1):
            ratio = i / float(steps)
            travel = direction * self.step_size * ratio
            sample_yaw = wrap_angle(yaw + travel * math.tan(steer) / max(self.wheel_base, 1e-6))
            avg_yaw = wrap_angle(0.5 * (yaw + sample_yaw))
            sample_x = x + travel * math.cos(avg_yaw)
            sample_y = y + travel * math.sin(avg_yaw)
            if not self._point_collision_free(sample_x, sample_y, grid_map):
                return False
        return True

    def _point_collision_free(self, x, y, grid_map):
        if callable(grid_map):
            return not bool(grid_map(x, y))

        bounds = grid_map.get("bounds") if isinstance(grid_map, dict) else None
        if bounds is not None:
            min_x, min_y, max_x, max_y = bounds
            if x < min_x or x > max_x or y < min_y or y > max_y:
                return False

        obstacles = grid_map.get("obstacles", []) if isinstance(grid_map, dict) else []
        for obstacle in obstacles:
            ox = float(obstacle[0])
            oy = float(obstacle[1])
            radius = float(obstacle[2]) if len(obstacle) > 2 else 0.5
            if math.hypot(x - ox, y - oy) <= radius:
                return False
        return True

    def _state_key(self, x, y, yaw):
        return (
            int(round(x / self.grid_resolution)),
            int(round(y / self.grid_resolution)),
            int(round(wrap_angle(yaw) / self.yaw_resolution)),
        )

    def _heuristic(self, x, y, gx, gy):
        return math.hypot(gx - x, gy - y)

    def _goal_reached(self, node, gx, gy, gyaw):
        distance_ok = math.hypot(gx - node.x, gy - node.y) <= max(2.0 * self.grid_resolution, self.step_size)
        yaw_ok = abs(wrap_angle(gyaw - node.yaw)) <= 2.0 * self.yaw_resolution
        return distance_ok and yaw_ok

    def _reconstruct(self, nodes, node_id, goal_xy):
        path = []
        while node_id is not None:
            node = nodes[node_id]
            path.append((node.x, node.y))
            node_id = node.parent
        path.reverse()
        path.append((float(goal_xy[0]), float(goal_xy[1])))
        return path
