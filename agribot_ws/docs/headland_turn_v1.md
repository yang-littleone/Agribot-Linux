# 地头 U 形换行第一版说明

本文档记录当前工程中新增的第一版地头掉头方案。该版本由中心线提取节点负责地头检测，由 PID 导航节点负责执行 U 形换行。

## 1. 功能目标

实现如下流程：

```text
ROW_FOLLOW
→ U_TURN
→ NEXT_ROW_REACQUIRE
→ ROW_FOLLOW
```

含义：

1. `ROW_FOLLOW`：正常跟踪 `/corn_row_center_line`。
2. `U_TURN`：检测到当前行前方中心线持续变弱或消失后，生成并跟踪一条几何 U 形路径。
3. `NEXT_ROW_REACQUIRE`：掉头完成后先跟踪短预测中心线；若检测到候选中心线，则按置信度融合预测中心线与检测中心线；连续高置信重捕获后恢复正常跟踪。
4. 重捕获成功后恢复正常行间跟踪。

## 2. 新增话题

```text
/headland_turn_path
```

类型：`nav_msgs/Path`

用途：显示当前生成的 U 形掉头路径。

```text
/navigation_mode
```

类型：`std_msgs/String`

可能取值：

```text
ROW_FOLLOW
U_TURN
NEXT_ROW_REACQUIRE
```

```text
/headland_detected
```

类型：`std_msgs/Bool`

用途：由 `corn_row_detector_projection` 根据中心线诊断结果发布地头检测结果，PID 控制器只订阅该布尔信号，不再直接解析诊断数组。

```text
/reacquire_reference_path
```

类型：`nav_msgs/Path`

用途：显示 `NEXT_ROW_REACQUIRE` 阶段控制器实际跟踪的参考线；可能是预测中心线，也可能是预测中心线与检测中心线的融合结果。

## 3. 关键参数

### 3.1 感知节点参数

以下参数属于：

```text
centerline_extraction corn_row_detector_projection
```

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `enable_headland_detection` | `true` | 是否启用地头感知检测 |
| `headland_candidate_frames` | `6` | 连续多少帧被判定为地头候选后发布 `/headland_detected=true` |
| `headland_low_confidence_threshold` | `0.35` | 低置信地头候选阈值 |
| `headland_min_side_points` | `80` | 左右任一侧内侧行点数低于该值时，认为行支持变弱 |
| `headland_min_path_points` | `5` | 中心线路径点数低于该值时，认为中心线过短 |

### 3.2 PID 执行参数

以下参数属于：

```text
centerline_extraction cornfield_navigation_node
```

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `enable_headland_turn` | `false` | 是否启用地头 U 形换行 |
| `headland_min_follow_distance` | `1.5` | 至少先正常行驶一小段距离后才允许感知触发，避免刚启动误判 |
| `headland_row_spacing` | `0.87` | 相邻行中心距，主要用于默认掉头半径 |
| `headland_turn_radius` | `0.0` | U 形半圆半径；为 0 时使用 `headland_row_spacing / 2` |
| `headland_exit_distance` | `0.4` | 掉头前继续向前驶出的距离 |
| `headland_settle_distance` | `0.6` | 掉头后沿新行方向对齐的距离 |
| `headland_path_step` | `0.08` | U 形路径采样间距 |
| `headland_turn_goal_tolerance` | `0.25` | 到达掉头终点的位置容差 |
| `headland_turn_heading_tolerance` | `0.6` | 到达掉头终点的航向容差，单位 rad |
| `headland_reacquire_confidence` | `0.75` | 相邻行重捕获要求的最低中心线置信度 |
| `headland_reacquire_track_confidence` | `0.35` | 重捕获阶段允许低速跟踪候选中心线的最低置信度 |
| `headland_reacquire_search_speed` | `0.08` | 重捕获阶段未稳定检测到中心线时的低速搜索线速度 |
| `headland_reacquire_search_angular_speed` | `0.0` | 重捕获阶段无候选中心线时的搜索角速度 |
| `headland_reacquire_max_distance` | `1.5` | 重捕获阶段最大允许搜索距离，超限停车 |
| `headland_reacquire_max_time` | `8.0` | 重捕获阶段最大允许搜索时间，超限停车 |
| `headland_reacquire_prediction_length` | `1.5` | 掉头完成后生成的预测中心线长度 |
| `headland_reacquire_frames` | `5` | 连续多少帧满足置信度后恢复行间跟踪 |
| `max_headland_turns` | `0` | 最多执行几次掉头；小于等于 0 表示不限次数 |
| `headland_turn_direction` | `left` | 第一次掉头方向；之后自动左右交替 |
| `headland_turn_linear_speed` | `0.22` | U 形掉头阶段最大线速度 |
| `headland_turn_target_distance` | `0.35` | U 形路径跟踪的前视目标距离 |

## 4. 启动示例

开启一次左侧换行，使用感知触发地头：

```bash
PID_EXTRA_ARGS="-p enable_headland_turn:=true \
-p headland_turn_direction:=left" \
./scripts/run_quality_pid_trial.sh turn_trial_01
```

如果旧目录已经存在，可以换 `OUT_ROOT`：

```bash
OUT_ROOT=$HOME/agribot_test_logs/headland_turn_v1 \
PID_EXTRA_ARGS="-p enable_headland_turn:=true \
-p headland_turn_direction:=left" \
./scripts/run_quality_pid_trial.sh trial_01
```

若第一次向左掉头，则后续方向自动为：

```text
left → right → left → right → ...
```

如果只想限制掉头次数，例如只测试 2 次：

```bash
PID_EXTRA_ARGS="-p enable_headland_turn:=true \
-p headland_turn_direction:=left \
-p max_headland_turns:=2" \
./scripts/run_quality_pid_trial.sh trial_01
```

如果需要调地头检测阈值，应给中心线提取节点传参。当前通用实验脚本只提供 `PID_EXTRA_ARGS`，因此更建议先手动三终端启动调试：

```bash
ros2 launch diff_drive_robot robot.launch.py world:=zhenshi4hang16m.world
ros2 run centerline_extraction corn_row_detector_projection --ros-args \
-p headland_candidate_frames:=6 \
-p headland_min_side_points:=80 \
-p headland_low_confidence_threshold:=0.35
ros2 run centerline_extraction cornfield_navigation_node --ros-args \
-p enable_headland_turn:=true \
-p headland_turn_direction:=left
```

## 5. RViz 建议查看

建议添加以下显示：

1. `Path`: `/corn_row_center_line`
2. `Path`: `/headland_turn_path`
3. `Path`: `/reacquire_reference_path`
4. `Marker`: `/target_point`
5. `PointCloud2`: `/point_cloud_projected`
6. `RobotModel`

同时可以命令行查看状态：

```bash
ros2 topic echo /navigation_mode
ros2 topic echo /headland_detected
```

## 6. 第一版限制

1. 地头触发基于中心线诊断话题，包括中心线有效性、左右行点数、路径点数和置信度；不是固定行长触发。
2. U 形路径是几何固定路径，不会主动避障。
3. 掉头阶段默认不使用中心线置信度停车，避免地头无中心线时立刻停车。
4. 掉头完成后会先沿预测中心线低速行驶，不再原地停车等待；检测中心线出现后通过置信度加权融合，只有连续满足高置信阈值后才恢复正常行间跟踪。
5. 若重获阶段超过最大距离或最大时间仍未稳定重获中心线，则停车等待人工处理或后续策略接管。
6. 地头空间不足或行距参数不准确时，可能无法顺利进入下一行。

## 7. 后续优化方向

1. 根据检测到的相邻行位置动态修正 U 形路径终点。
2. 在 U 形路径跟踪中加入障碍物或安全裕度约束。
3. 根据不同地头形态自适应选择左转、右转或原路返回。
