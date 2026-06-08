# Unity 智能车控制 baseline 改造说明

## 文件变化

- `main.py`：保留 `Control` 类和 `control_node` 主循环，改为初始化生成参考轨迹、25Hz 轻量跟踪控制。
- `path_utils.py`：JSON 路径加载、闭环识别、重采样、yaw/curvature 计算、曲率速度规划、最近点查询。
- `planner.py`：`HybridAStarPlanner` 可插拔接口；有地图时可低频搜索，无地图时自动 fallback 到 JSON 路径。
- `optimizer.py`：轻量 MPC-like 离线轨迹平滑器，降低航向、曲率和曲率变化突变。
- `controller.py`：Pure Pursuit 横向控制和 PID 纵向速度控制。
- `debug_logger.py`：限频输出调试量，避免控制循环刷屏。
- `my_udp.py`：未改通信协议，仍通过 `send_control_command(v, w)` 发送 `{"vx": v, "vz": w}`。

## 主控制流程

启动后 `Control.load_route()` 读取 JSON 路径，经过 Hybrid A* fallback、重采样、MPC-like 平滑、曲率计算和速度规划，得到包含 `x/y/yaw/curvature/target_speed` 的参考轨迹。

25Hz 控制循环只做轻量计算：读取 UDP 车辆状态，更新最近轨迹点，读取该点目标速度，PID 生成 `v_cmd`，Pure Pursuit 生成转向角并转换为 `w_cmd`，最后调用 `self.udp_client.send_control_command(v_cmd, w_cmd)`。

## Fallback 与平滑

当前没有道路边界或障碍地图时，`HybridAStarPlanner.plan()` 自动使用已有 JSON 路径作为粗路径；如果后续提供 `grid_map`、起点和终点，可启用简化 Hybrid A*。

RS/G2 连续化采用可运行近似方案：对粗路径重采样后使用二阶平滑项和四阶曲率变化项做 MPC-like 离线优化，再统一计算 yaw 和 curvature。这样不依赖 scipy/cvxpy，也能减少连接处曲率突变和转向抖动。

## 控制与限速

速度规划按曲率降速：直道接近 `18.0m/s`，弯道降到 `5.0m/s` 附近，并经过加减速度约束和低通滤波。代码中 `max_speed=18.0`，`hard_speed_limit=19.0`，所有 `v_cmd` 再次限幅到 `<20m/s` 安全裕度内。

Pure Pursuit 使用动态预瞄距离：低速短预瞄提高贴线精度，高速长预瞄提高稳定性；转向角、角速度和转向变化率都有限幅。PID 用当前 `vehicle_data.speed` 和轨迹 `target_speed` 计算最终速度命令，并限制积分、输出和加速度变化。

## 日志与轨迹记录

默认启动时会写两类文件：`log/control_*.csv` 每 3 帧记录一次控制调试量，`run_route/run_route_*.json` 记录车辆实际运动轨迹。实际轨迹 JSON 与 `Big.json` 一致，格式为 `[{"x": ..., "y": ...}, ...]`，可直接作为路线文件加载。轨迹文件启动时立即创建，并默认每 3 帧原子保存一次，程序正常退出或收到 `SIGINT/SIGTERM` 时会再保存一次。

常用命令：

- `python3 main.py --disable-control-log`：关闭控制调试日志。
- `python3 main.py --disable-trajectory-log`：关闭实际轨迹记录。
- `python3 main.py --enable-control-log --enable-trajectory-log --log-every-frames 3 --route-save-every-frames 3 --log-dir log --run-route-dir run_route`：显式开启两类记录。
- `python3 main.py --route-file exp_routes/Big.json --net-config name,127.0.0.1,8448,8449`：命令行指定路线和 UDP 网络配置。

## 调参建议

1. 过弯切内侧：降低 `max_speed` 或 `max_lateral_accel`，增大 `lookahead_base/lookahead_gain`，或增大 `smoothing_data_weight` 让轨迹更贴近原始路线。
2. 左右摇摆：增大 `lookahead_gain`，降低 `max_steer_rate` 或 `max_yaw_rate`，适当提高 `smoothing_curvature_weight`。
3. 弯道速度过快：降低 `min_speed`、`max_lateral_accel`，或增大 `curvature_speed_gain`。
4. 控制延迟或不稳定：确认 Unity 状态频率稳定，保持 `control_rate=25Hz`；降低 `pid_kp/pid_kd`，提高预瞄距离，适当降低 `max_accel`。
