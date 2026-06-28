# PID Path Tracking Accuracy Test

本文档记录用于单独验证 PID 路径跟踪精度的测试程序。该测试绕开中心线提取算法，直接向 `/corn_row_center_line` 发布一条已知参考路径，从而评估 `cornfield_navigation_node` 中 PID 控制器本身的跟踪准确性。

## 测试程序

脚本位置：

```text
scripts/pid_path_tracking_accuracy_test.py
```

脚本功能：

1. 启动 Gazebo 仿真。
2. 启动 `cornfield_navigation_node`。
3. 持续发布一条包含直线、圆弧、直线的参考路径。
4. 订阅 `/odom` 记录机器人真实轨迹。
5. 判断机器人是否到达参考路径最后一个点附近，到达后发布零速度并停止测试。
6. 计算机器人轨迹到参考路径的横向误差和航向误差。
7. 输出 CSV 表格、Markdown 报告和验证图片。

## 参考路径

参考路径由三段组成：

- `straight_1`: 沿 `+x` 方向直线，长度 4 m。
- `arc`: 半径 2 m 的左转 90 度圆弧。
- `straight_2`: 圆弧后沿 `+y` 方向直线，长度 3 m。

总参考路径长度约为：

```text
10.142 m
```

## 运行命令

推荐先使用无 GUI 模式运行：

```bash
./scripts/pid_path_tracking_accuracy_test.py \
  --gui false \
  --duration 70 \
  --goal-tolerance 0.25 \
  --progress-tolerance 0.20 \
  --trial-name trial_01 \
  --output-root /home/xkai/agribot_test_logs/pid_tracking_accuracy
```

默认使用：

```text
world = empty.world
use_quality_aware_control = false
max_linear_speed = 0.35
target_distance = 0.45
stop_when_complete = true
goal_tolerance = 0.25
progress_tolerance = 0.20
```

停止逻辑：

- 当机器人投影到参考路径上的进度到达路径末端附近；
- 且机器人当前位置距离最后一个参考点小于 `goal_tolerance`；
- 脚本立即发布零速度 `/cmd_vel` 并结束记录。

`duration` 不是主要停止条件，只作为超时保护，防止 PID 无法到达终点时测试无限运行。

如果要打开 Gazebo GUI：

```bash
./scripts/pid_path_tracking_accuracy_test.py --gui true --duration 70
```

## 输出文件

样例输出目录：

```text
/home/xkai/agribot_test_logs/pid_tracking_accuracy/trial_01
```

主要输出：

```text
tracking_accuracy_report.md
tracking_metrics.csv
tracking_samples.csv
tracking_trajectory.png
cross_track_error.png
heading_error.png
gazebo.log
navigation_pid.log
```

其中：

- `tracking_accuracy_report.md`: 汇总报告。
- `tracking_metrics.csv`: 整体和分段误差指标。
- `tracking_samples.csv`: 每个 odom 采样点对应的最近路径点和误差。
- `tracking_trajectory.png`: 参考路径与机器人轨迹对比图。
- `cross_track_error.png`: 横向误差随时间变化曲线。
- `heading_error.png`: 航向误差随时间变化曲线。

## 当前样例结果

样例命令：

```bash
./scripts/pid_path_tracking_accuracy_test.py \
  --gui false \
  --duration 70 \
  --trial-name trial_01 \
  --output-root /home/xkai/agribot_test_logs/pid_tracking_accuracy
```

样例结果：

| 指标 | 数值 |
|---|---:|
| 采样点数 | 2057 |
| 测试时长 | 69.979 s |
| 路径完成度 | 99.7% |
| 横向误差 MAE | 0.3358 m |
| 横向误差 RMSE | 0.7521 m |
| 横向误差 Max | 2.6310 m |
| 横向误差 P95 | 2.0905 m |
| 航向误差 MAE | 24.156 deg |
| 航向误差 RMSE | 46.852 deg |

分段结果：

| 路段 | CTE mean/m | CTE RMSE/m | CTE max/m | Heading mean/deg |
|---|---:|---:|---:|---:|
| straight_1 | 0.0021 | 0.0032 | 0.0068 | 0.316 |
| arc | 0.0297 | 0.0309 | 0.0457 | 2.160 |
| straight_2 | 0.6325 | 1.0441 | 2.6310 | 45.409 |

## 初步判断

当前 PID 在第一段直线和圆弧段跟踪较好：

```text
straight_1 RMSE = 0.0032 m
arc RMSE = 0.0309 m
```

但在圆弧后的第二段直线误差明显增大：

```text
straight_2 RMSE = 1.0441 m
straight_2 max error = 2.6310 m
```

这说明当前 PID 对简单直线和圆弧入口具有较好跟踪能力，但在圆弧结束后切换到新方向直线时存在较大偏差，可能原因包括：

- 目标点选择策略对路径末端和大曲率切换不够稳健。
- 当前 PID 参数对 90 度转向后的收敛能力不足。
- `find_target_point()` 中路径末端处理逻辑可能导致机器人在路径末端附近继续向自身前方追踪，而不是稳定贴合参考路径末段。

后续若要优化 PID，建议优先针对圆弧后直线段的收敛性能进行调参或改进目标点选择策略。
