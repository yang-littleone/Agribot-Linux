# PID Quality-Aware Control Experiment Record

本文档用于记录当前阶段已经完成的 PID 对照实验，作为后续修改代码前的基线说明。记录日期：2026-06-28。

## 实验目的

验证在冠下玉米行仿真场景中，将走廊置信度 `corridor_confidence` 和安全裕度 `corridor_safety_margin` 接入 PID 路径跟踪控制器后，是否相比普通 PID 具有更好的连续通行能力和安全性。

本轮实验重点不是单独证明中心线提取算法精度，而是比较两种控制策略：

- `quality_aware`: 启用质量感知控制，`use_quality_aware_control=true`。
- `baseline_pid`: 关闭质量感知控制，`use_quality_aware_control=false`。

## 测试环境

- 工作空间：`/home/xkai/agribot/agribot_ws`
- 世界文件：`zhenshi4hang16m.world`
- 启动仿真：

```bash
ros2 launch diff_drive_robot robot.launch.py world:=zhenshi4hang16m.world
```

- 启动中心线提取：

```bash
ros2 run centerline_extraction corn_row_detector_projection
```

- 启动路径跟踪：

```bash
ros2 run centerline_extraction cornfield_navigation_node
```

实际测试中使用统一脚本自动完成上述流程和 rosbag 记录：

```bash
./scripts/run_quality_pid_trial.sh trial_01
```

脚本位置：

```text
scripts/run_quality_pid_trial.sh
```

脚本会自动记录以下主要话题：

```text
/odom
/cmd_vel
/target_point
/mid360_PointCloud2
/corn_row_center_line
/corn_row_center_line_viz
/under_canopy_left_boundary
/under_canopy_right_boundary
/corridor_width
/corridor_safety_margin
/corridor_confidence
/point_cloud_projected
/left_row_points
/right_row_points
```

## 质量感知 PID 组

质量感知组使用默认参数：

```bash
OUT_ROOT=$HOME/agribot_test_logs/zhenshi4hang16m_quality_pid \
./scripts/run_quality_pid_trial.sh trial_01
```

其中 PID 参数：

```text
use_quality_aware_control=true
```

数据目录：

```text
/home/xkai/agribot_test_logs/zhenshi4hang16m_quality_pid/trial_01
/home/xkai/agribot_test_logs/zhenshi4hang16m_quality_pid/trial_02
/home/xkai/agribot_test_logs/zhenshi4hang16m_quality_pid/trial_03
/home/xkai/agribot_test_logs/zhenshi4hang16m_quality_pid/trial_04
/home/xkai/agribot_test_logs/zhenshi4hang16m_quality_pid/trial_05
```

每次运行时长约 300 s，五次均为有效实验。

## 普通 PID 对照组

普通 PID 组关闭质量感知控制：

```bash
OUT_ROOT=$HOME/agribot_test_logs/zhenshi4hang16m_pid_baseline \
PID_EXTRA_ARGS="-p use_quality_aware_control:=false" \
./scripts/run_quality_pid_trial.sh trial_01
```

其中 PID 参数：

```text
use_quality_aware_control=false
```

数据目录：

```text
/home/xkai/agribot_test_logs/zhenshi4hang16m_pid_baseline/trial_01
/home/xkai/agribot_test_logs/zhenshi4hang16m_pid_baseline/trial_02
/home/xkai/agribot_test_logs/zhenshi4hang16m_pid_baseline/trial_03
/home/xkai/agribot_test_logs/zhenshi4hang16m_pid_baseline/trial_04
/home/xkai/agribot_test_logs/zhenshi4hang16m_pid_baseline/trial_05
```

每次运行时长约 300 s，五次均为有效实验。

## 相关代码改动

PID 控制器中新增参数：

```text
use_quality_aware_control
```

参数含义：

- `true`: 启用走廊置信度、安全裕度、历史中心线恢复、低质量阶段降速/停车逻辑。
- `false`: 关闭上述质量感知控制逻辑，退化为普通中心线 PID 跟踪。

相关文件：

```text
src/centerline_extraction/src/pid_controller.cpp
src/centerline_extraction/include/centerline_extraction/pid_controller.hpp
scripts/run_quality_pid_trial.sh
```

脚本新增环境变量：

```text
PID_EXTRA_ARGS
```

用于在不修改脚本的情况下给 `cornfield_navigation_node` 传入额外 ROS 参数。

## 已生成结果文件

质量感知 PID 五次重复实验结果：

```text
docs/zhenshi4hang16m_five_trial_results.md
docs/zhenshi4hang16m_five_trial_results.csv
docs/figures/zhenshi4hang16m_five_trials/README.md
```

质量感知 PID 与普通 PID 对照分析：

```text
docs/zhenshi4hang16m_quality_vs_baseline_analysis.md
docs/zhenshi4hang16m_quality_vs_baseline_metrics.csv
```

## 主要结果

两组均完成 5/5 次有效实验，说明普通 PID 与质量感知 PID 在固定 `zhenshi4hang16m.world` 场景下均具备通过能力。

核心对照结果如下：

| 指标 | 质量感知 PID | 普通 PID | 说明 |
|---|---:|---:|---|
| 行驶距离 | 16.264 m | 16.289 m | 两者均完成约 16 m 通行 |
| 终点横向绝对误差 | 0.004 m | 0.005 m | 差异很小 |
| 全程横向 RMSE | 0.008 m | 0.008 m | 基本相同 |
| 最大横向绝对偏差 | 0.017 m | 0.017 m | 基本相同 |
| 平均线速度 | 0.199 m/s | 0.177 m/s | 质量感知 PID 更连续 |
| 速度为零比例 | 19.191% | 38.124% | 质量感知 PID 明显减少停车 |
| 平均走廊置信度 | 0.739 | 0.568 | 质量感知组感知状态更稳定 |
| 平均安全裕度 | 0.186 m | 0.142 m | 质量感知组安全裕度更高 |
| 中线可用率 | 81.009% | 62.017% | 质量感知组跟踪连续性更好 |

95% 置信区间中较有力的指标：

```text
速度为零比例差值: -18.933%
95% CI: [-30.228%, -7.638%]
```

该区间不跨 0，说明质量感知 PID 对减少停车/停顿有较明确提升。

## 结果解释

本轮实验不能强行表述为“质量感知 PID 显著降低横向跟踪误差”。因为终点误差、RMSE 和最大横向偏差两组都在厘米级，且差异很小。

更准确的结论是：

```text
质量感知 PID 在保持相近横向跟踪精度的基础上，显著降低了低质量感知阶段的停车比例，提高了连续通行能力和安全裕度。
```

原因是普通 PID 关闭质量感知后，虽然不主动根据置信度降速，但当中心线为空时仍会停车；质量感知 PID 可以在低置信度但未完全失效时使用历史中心线恢复跟踪，因此整体停顿更少。

## 可写入论文的表述

在相同冠下仿真场景中，相比不使用感知质量信息的普通 PID，质量感知 PID 通过引入走廊置信度、安全裕度和历史中心线恢复机制，在保持相近横向跟踪误差的同时，将速度为零比例由 38.124% 降低至 19.191%，并提高了中线可用率和平均安全裕度。结果表明，所提出的感知质量约束控制策略主要优势不是显著降低厘米级横向误差，而是减少低质量感知阶段的停顿，提高连续通行能力和运行安全性。

## 后续改动前注意事项

后续如果继续修改中心线提取算法或 PID 控制器，应保留当前两组结果作为对照基线。新的代码改动完成后，建议再次执行：

1. 同一世界 `zhenshi4hang16m.world` 下 5 次质量感知 PID 测试。
2. 同一世界 `zhenshi4hang16m.world` 下 5 次普通 PID 测试。
3. 重新生成 `quality_vs_baseline` 分析表。
4. 比较速度为零比例、安全裕度、中线可用率、横向 RMSE 和终点横向误差。

如果后续要增强论文说服力，还需要补充：

- 不同初始偏航角实验。
- 不同玉米行密度或冠层遮挡程度实验。
- 不同速度参数实验。
- 不同中心线提取算法或不同拟合策略对比实验。

