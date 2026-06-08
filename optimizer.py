import math


class MPCLikeOptimizer:
    """Small offline path smoother used before the realtime control loop."""

    def __init__(
        self,
        iterations=80,
        data_weight=0.08,
        smooth_weight=0.28,
        curvature_weight=0.04,
        max_step=0.12,
        tolerance=1e-5,
    ):
        self.iterations = int(iterations)
        self.data_weight = float(data_weight)
        self.smooth_weight = float(smooth_weight)
        self.curvature_weight = float(curvature_weight)
        self.max_step = float(max_step)
        self.tolerance = float(tolerance)

    def optimize(self, xs, ys, closed=False):
        if len(xs) < 4:
            return list(xs), list(ys)

        original_x = [float(v) for v in xs]
        original_y = [float(v) for v in ys]
        smooth_x = original_x[:]
        smooth_y = original_y[:]
        n = len(smooth_x)

        for _ in range(self.iterations):
            prev_x = smooth_x[:]
            prev_y = smooth_y[:]
            max_change = 0.0

            indices = range(n) if closed else range(1, n - 1)
            for i in indices:
                im1 = (i - 1) % n if closed else max(0, i - 1)
                ip1 = (i + 1) % n if closed else min(n - 1, i + 1)
                im2 = (i - 2) % n if closed else max(0, i - 2)
                ip2 = (i + 2) % n if closed else min(n - 1, i + 2)

                lap_x = prev_x[im1] + prev_x[ip1] - 2.0 * prev_x[i]
                lap_y = prev_y[im1] + prev_y[ip1] - 2.0 * prev_y[i]

                curvature_grad_x = (
                    -prev_x[im2] + 4.0 * prev_x[im1] - 6.0 * prev_x[i]
                    + 4.0 * prev_x[ip1] - prev_x[ip2]
                )
                curvature_grad_y = (
                    -prev_y[im2] + 4.0 * prev_y[im1] - 6.0 * prev_y[i]
                    + 4.0 * prev_y[ip1] - prev_y[ip2]
                )

                dx = (
                    self.data_weight * (original_x[i] - prev_x[i])
                    + self.smooth_weight * lap_x
                    + self.curvature_weight * curvature_grad_x
                )
                dy = (
                    self.data_weight * (original_y[i] - prev_y[i])
                    + self.smooth_weight * lap_y
                    + self.curvature_weight * curvature_grad_y
                )

                step = math.hypot(dx, dy)
                if step > self.max_step > 0.0:
                    scale = self.max_step / step
                    dx *= scale
                    dy *= scale

                smooth_x[i] = prev_x[i] + dx
                smooth_y[i] = prev_y[i] + dy
                max_change = max(max_change, abs(dx), abs(dy))

            if max_change < self.tolerance:
                break

        return smooth_x, smooth_y
