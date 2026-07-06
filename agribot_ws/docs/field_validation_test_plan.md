# 冠下导航实地验证测试计划

本文档用于指导 `agribot_ws` 项目的实地验证工作，目标是把当前仿真中的冠下可通行走廊检测、置信度/安全裕度评价和质量感知 PID 控制逐步验证到真实玉米冠下环境中。

推荐验证路线为：

```text
准备与检查
-> 人工遥控采 rosbag
-> 离线真实点云感知验证
-> 小范围低速半闭环验证
-> 完整行间闭环对照实验
-> 数据统计与论文材料整理
```

第一篇论文建议优先完成前三个阶段：真实点云感知验证 + 小范围低速闭环可行性验证。完整地头换行暂不作为主验证内容。

## 1. 总体目标

### 1.1 感知验证目标

验证 `corn_row_detector_projection` 在真实玉米冠下点云中能否稳定输出：

```text
/corn_row_center_line
/corn_row_center_line_viz
/under_canopy_left_boundary
/under_canopy_right_boundary
/corridor_width
/corridor_safety_margin
/corridor_confidence
/centerline_detection_diagnostics
/left_row_points
/right_row_points
/point_cloud_projected
```

重点回答：

1. 中心线是否位于真实行间中部。
2. 左右边界是否对应当前通道两侧最近内侧作物行。
3. 车体偏航、多行干扰、叶片遮挡和局部缺株时，检测是否仍稳定。
4. 置信度低时是否真的对应点云缺失、遮挡严重或通道估计不可靠。
5. 安全裕度是否能反映通道变窄和作物侵入风险。

### 1.2 控制验证目标

验证质量感知 PID 相比普通 PID 是否能在真实行间：

1. 保持相近横向跟踪误差。
2. 降低中心线低质量阶段的停车比例。
3. 减少碰株、擦碰和人工接管。
4. 提高连续通行能力和安全裕度。

### 1.3 论文证据链

建议最终形成三层证据：

```text
仿真多工况实验
    证明算法机制和对照结果

真实点云离线实验
    证明感知算法能泛化到真实冠下点云

实车低速闭环实验
    证明系统具备真实行间通行可行性
```

## 2. 实验周期划分

### 周期 A：出发前准备

目标：确保实车、传感器、ROS2 话题、TF、录包脚本和安全机制都可用。

建议时间：实地前 2-3 天。

### 周期 B：现场人工遥控采 rosbag

目标：不让自动控制器接管，只由人工遥控小车沿行间行驶，记录真实点云和算法输出。

建议时间：第 1 次实地主要做这一阶段。

### 周期 C：离线真实点云感知验证

目标：回放 rosbag，分析中心线、边界、宽度、置信度和安全裕度。

建议时间：采包当天晚上或次日。

### 周期 D：小范围低速半闭环验证

目标：在 3-5 m 短距离内低速启用自动控制，人随时接管。

建议时间：确认周期 C 结果可靠后再做。

### 周期 E：完整行间闭环对照实验

目标：在 10-20 m 行间进行普通 PID 与质量感知 PID 对照。

建议时间：周期 D 稳定后进行。

### 周期 F：数据分析与论文整理

目标：生成论文可用表格、曲线、统计指标、图像和实验描述。

## 3. 周期 A：出发前准备

### 3.1 代码与工作空间检查

在工作站上执行：

```bash
cd /home/xkai/agribot/agribot_ws
colcon build --symlink-install
source install/setup.bash
```

检查核心节点是否存在：

```bash
ros2 pkg executables centerline_extraction
```

应至少能看到：

```text
corn_row_detector_projection
cornfield_navigation_node
```

### 3.2 雷达话题检查

当前感知节点默认订阅：

```text
/mid360_PointCloud2
```

实车启动雷达后检查：

```bash
ros2 topic list | grep -i point
ros2 topic hz /mid360_PointCloud2
ros2 topic echo /mid360_PointCloud2 --once
```

如果实车雷达话题不是 `/mid360_PointCloud2`，现场运行时使用 remap：

```bash
ros2 run centerline_extraction corn_row_detector_projection --ros-args \
  -r /mid360_PointCloud2:=/your_livox_topic
```

待完成事项：

- [ ] 确认实车雷达点云话题名。
- [ ] 确认点云频率，建议不低于 3 Hz。
- [ ] 确认点云 frame_id。
- [ ] 确认点云在 RViz 中方向正确。

### 3.3 TF 检查

感知节点需要把雷达点云变换到 `base_link`。必须确认：

```text
lidar_frame -> base_link
base_link -> odom
```

或等价 TF 链路可用。

检查命令：

```bash
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo base_link your_lidar_frame
ros2 run tf2_ros tf2_echo odom base_link
```

待完成事项：

- [ ] 确认雷达到 `base_link` 的外参正确。
- [ ] 确认 `odom` 到 `base_link` 连续发布。
- [ ] 确认 `base_link` 的 x 轴朝车头前方，y 轴朝左。
- [ ] 确认点云投影后左右作物行不会反向。

### 3.4 里程计检查

检查：

```bash
ros2 topic hz /odom
ros2 topic echo /odom --once
```

人工推动或遥控小车前进，确认：

1. 前进时 `x` 方向增加或至少轨迹连续。
2. 原地转向时 yaw 变化合理。
3. 里程计不会明显跳变。

待完成事项：

- [ ] 确认 `/odom` 可用。
- [ ] 确认 `/odom` frame_id 和 child_frame_id 合理。
- [ ] 确认短距离 10-20 m 内漂移可接受。

### 3.5 安全检查

必须具备：

1. 急停开关。
2. 遥控器或键盘接管。
3. 低速限制。
4. 人员站位安全。
5. 电池电量充足。

实车闭环前建议控制参数：

```text
max_linear_speed: 0.05-0.15 m/s
min_linear_speed: 0.02-0.05 m/s
max_angular_speed: 0.3-0.6 rad/s
safety_margin_stop: 0.08-0.10 m
```

待完成事项：

- [ ] 测试急停有效。
- [ ] 测试遥控接管有效。
- [ ] 确认自动控制不启动时，人工遥控正常。
- [ ] 确认 `/cmd_vel` 不会被多个节点同时抢占。

### 3.6 录包目录规范

建议数据统一放入：

```text
~/agribot_field_logs/
```

目录命名：

```text
field_YYYYMMDD_siteXX/
```

单次试验命名：

```text
normal_trial_01
normal_trial_02
leaf_occlusion_trial_01
missing_plants_trial_01
left_yaw_trial_01
right_yaw_trial_01
```

每次试验额外记录一个 `notes.md`，内容包括：

```text
日期：
地点：
作物：
生育期/高度：
行距：
行长：
天气：
地面情况：
雷达安装高度：
雷达俯仰角：
车体宽度：
操作方式：
试验场景：
异常情况：
```

## 4. 周期 B：现场人工遥控采 rosbag

### 4.1 目的

人工遥控采 rosbag 的意思是：

```text
车不自动导航
人用遥控器/键盘/手柄开车
ROS2 记录真实雷达点云、TF、里程计和算法输出
回来后用这些数据验证中心线/走廊检测算法
```

这一阶段只验证感知算法，不验证 PID 控制效果。

### 4.2 现场启动顺序

终端 1：启动实车底盘、雷达、TF 和里程计。

具体命令根据实车 launch 决定。启动后检查：

```bash
ros2 topic list
ros2 topic hz /mid360_PointCloud2
ros2 topic hz /odom
```

终端 2：启动中心线检测节点。

```bash
cd /home/xkai/agribot/agribot_ws
source install/setup.bash
ros2 run centerline_extraction corn_row_detector_projection
```

如果需要 remap：

```bash
ros2 run centerline_extraction corn_row_detector_projection --ros-args \
  -r /mid360_PointCloud2:=/your_livox_topic
```

终端 3：启动 RViz 观察。

建议添加：

```text
PointCloud2: /point_cloud_projected
PointCloud2: /left_row_points
PointCloud2: /right_row_points
Path: /corn_row_center_line
Path: /corn_row_center_line_viz
Path: /under_canopy_left_boundary
Path: /under_canopy_right_boundary
TF
RobotModel
```

终端 4：录包。

```bash
mkdir -p ~/agribot_field_logs/field_YYYYMMDD_site01
cd ~/agribot_field_logs/field_YYYYMMDD_site01

ros2 bag record \
  /mid360_PointCloud2 \
  /tf \
  /tf_static \
  /odom \
  /corn_row_center_line \
  /corn_row_center_line_viz \
  /under_canopy_left_boundary \
  /under_canopy_right_boundary \
  /corridor_width \
  /corridor_safety_margin \
  /corridor_confidence \
  /centerline_detection_diagnostics \
  /left_row_points \
  /right_row_points \
  /point_cloud_projected \
  -o normal_trial_01
```

如果雷达话题被 remap，则录原始真实话题和算法使用的话题中至少一个。推荐两者都录：

```bash
ros2 bag record \
  /your_livox_topic \
  /mid360_PointCloud2 \
  /tf \
  /tf_static \
  /odom \
  /corn_row_center_line \
  /corn_row_center_line_viz \
  /under_canopy_left_boundary \
  /under_canopy_right_boundary \
  /corridor_width \
  /corridor_safety_margin \
  /corridor_confidence \
  /centerline_detection_diagnostics \
  /left_row_points \
  /right_row_points \
  /point_cloud_projected \
  -o normal_trial_01
```

### 4.3 每次采集流程

每条数据按以下步骤执行：

1. 把机器人放在行间起点。
2. 确认车头大致朝行方向。
3. 开始录包。
4. 原地静止 5-10 s，用于记录初始点云。
5. 人工遥控低速前进，建议 `0.05-0.15 m/s`。
6. 行驶 10-30 m，或记录 60-180 s。
7. 停车后继续录 3-5 s。
8. 停止 rosbag。
9. 立刻记录现场 notes。

### 4.4 必采场景

建议第一轮至少采以下场景：

| 场景 | 次数 | 目的 |
|---|---:|---|
| 正常直行 | 5 | 验证基本中心线稳定性 |
| 叶片遮挡严重 | 5 | 验证置信度和历史保持 |
| 局部缺株/稀疏 | 5 | 验证点云缺失鲁棒性 |
| 初始左偏航 | 3-5 | 验证全局行向约束 |
| 初始右偏航 | 3-5 | 验证全局行向约束 |
| 多行可见/外侧行干扰 | 3-5 | 验证最近内侧行选择 |

如果时间不足，最低要求：

```text
正常直行 5 次
叶片遮挡 3 次
初始偏航 3 次
```

### 4.5 现场质量检查

每录完一条，快速检查：

```bash
ros2 bag info normal_trial_01
```

确认包含：

```text
/mid360_PointCloud2
/tf
/tf_static
/odom
/corn_row_center_line
/corridor_confidence
/centerline_detection_diagnostics
```

如果包内没有算法输出，至少要保证有：

```text
/mid360_PointCloud2
/tf
/tf_static
/odom
```

这样回来仍可离线重跑算法。

## 5. 周期 C：离线真实点云感知验证

### 5.1 回放方式 1：直接查看已录算法输出

如果现场录包时已经启动了 `corn_row_detector_projection`，可直接回放并查看输出：

```bash
cd ~/agribot_field_logs/field_YYYYMMDD_site01
ros2 bag play normal_trial_01
```

另开 RViz 查看：

```text
/point_cloud_projected
/left_row_points
/right_row_points
/corn_row_center_line
/under_canopy_left_boundary
/under_canopy_right_boundary
```

### 5.2 回放方式 2：只录原始数据，离线重跑算法

如果现场只录了原始点云、TF、odom，回放时重跑算法：

终端 1：

```bash
ros2 bag play normal_trial_01 --clock
```

终端 2：

```bash
cd /home/xkai/agribot/agribot_ws
source install/setup.bash
ros2 run centerline_extraction corn_row_detector_projection --ros-args \
  -p use_sim_time:=true
```

如需 remap：

```bash
ros2 run centerline_extraction corn_row_detector_projection --ros-args \
  -p use_sim_time:=true \
  -r /mid360_PointCloud2:=/your_livox_topic
```

### 5.3 感知指标

基于 `/centerline_detection_diagnostics` 统计：

```text
valid_ratio_pct             中心线有效率
center_offset_rmse_m        中心线横向偏移 RMSE
center_offset_std_m         中心线横向抖动
row_yaw_std_deg             行向角抖动
confidence_mean             平均置信度
confidence_low_ratio_pct    低置信度比例
safety_margin_mean_m        平均安全裕度
safety_margin_low_ratio_pct 低安全裕度比例
corridor_width_mean_m       平均走廊宽度
hold_ratio_pct              历史模型保持比例
path_points_mean            平均路径点数
```

项目已有脚本：

```text
scripts/analyze_centerline_diagnostics_trials.py
```

如果 rosbag 目录结构符合脚本要求，可使用该脚本生成统计表和曲线。若现场数据目录不同，需要稍微调整输入路径。

### 5.4 人工真值标注

SCI 论文最好补充部分人工真值。建议抽取每个场景 100-300 帧，人工标注：

```text
左内侧边界 y_l
右内侧边界 y_r
中心线 y_c = (y_l + y_r) / 2
作物行方向角 theta
```

可选真值来源：

1. RViz 中点云人工标注。
2. 俯视视频或无人机图像标注行中心。
3. 实地每隔 1 m 用卷尺测左右边界。
4. 规则直行场景中，用已知行距和起点中心建立近似真值。

建议论文指标：

```text
centerline_error_rmse_m
left_boundary_error_rmse_m
right_boundary_error_rmse_m
corridor_width_error_m
row_yaw_error_deg
```

待完成事项：

- [ ] 确定人工标注工具或流程。
- [ ] 每类场景至少标注 100 帧。
- [ ] 保存标注 CSV。
- [ ] 写脚本对齐算法输出与人工标注。

### 5.5 算法对照和消融

建议对比：

| 方法 | 说明 |
|---|---|
| 普通左右直线拟合 | 基础基线 |
| RANSAC 或鲁棒直线拟合 | 随机离群点鲁棒基线 |
| 当前全局行向 + 内侧行选择 | 主方法 |
| 主方法去掉全局行向 | 消融偏航解耦 |
| 主方法去掉内侧行选择 | 消融多行干扰 |
| 主方法去掉时序保持 | 消融稳定性 |
| 主方法去掉分段走廊 | 消融局部遮挡 |

当前代码中部分基线可能还没有独立开关，需要后续整理参数或单独实现。

待完成事项：

- [ ] 增加或整理基线算法启动参数。
- [ ] 固定同一 rosbag 重跑所有算法。
- [ ] 生成统一评价表。
- [ ] 生成中心线可视化对比图。

## 6. 周期 D：小范围低速半闭环验证

### 6.1 前置条件

只有满足以下条件，才进入半闭环：

- [ ] 真实点云中中心线基本稳定。
- [ ] 左右边界方向正确。
- [ ] 置信度低值与现场不可靠情况一致。
- [ ] 安全裕度没有明显系统性错误。
- [ ] 急停和遥控接管可靠。

### 6.2 半闭环含义

半闭环指：

```text
自动控制器发布 /cmd_vel
机器人低速行驶
人站在旁边，随时急停或遥控接管
测试距离短，优先验证可行性和安全
```

### 6.3 启动方式

终端 1：实车底盘、雷达、TF、odom。

终端 2：中心线检测。

```bash
ros2 run centerline_extraction corn_row_detector_projection
```

终端 3：质量感知 PID，保守限速。

```bash
ros2 run centerline_extraction cornfield_navigation_node --ros-args \
  -p use_quality_aware_control:=true \
  -p max_linear_speed:=0.10 \
  -p min_linear_speed:=0.03 \
  -p max_angular_speed:=0.50 \
  -p safety_margin_stop:=0.10 \
  -p safety_margin_mid:=0.15 \
  -p safety_margin_high:=0.25
```

终端 4：录包。

```bash
ros2 bag record \
  /mid360_PointCloud2 \
  /tf \
  /tf_static \
  /odom \
  /cmd_vel \
  /target_point \
  /corn_row_center_line \
  /corn_row_center_line_viz \
  /under_canopy_left_boundary \
  /under_canopy_right_boundary \
  /corridor_width \
  /corridor_safety_margin \
  /corridor_confidence \
  /centerline_detection_diagnostics \
  /left_row_points \
  /right_row_points \
  /point_cloud_projected \
  -o semi_closed_quality_pid_trial_01
```

### 6.4 测试步骤

每次测试：

1. 机器人放在行间中心。
2. 前方清空 3-5 m。
3. 人员站在机器人侧后方。
4. 启动感知节点，观察中心线稳定。
5. 启动 PID 控制器。
6. 行驶 3-5 m 后人工停止或急停。
7. 记录是否碰株、是否偏离、是否人工接管。

首次只跑：

```text
质量感知 PID 3 次
```

稳定后再跑：

```text
普通 PID 5 次
质量感知 PID 5 次
```

普通 PID 启动：

```bash
ros2 run centerline_extraction cornfield_navigation_node --ros-args \
  -p use_quality_aware_control:=false \
  -p max_linear_speed:=0.10 \
  -p min_linear_speed:=0.03 \
  -p max_angular_speed:=0.50
```

### 6.5 半闭环评价指标

```text
success_rate              成功通过率
manual_takeover_count     人工接管次数
crop_contact_count        碰株/擦碰次数
travel_distance_m         行驶距离
mean_abs_lateral_error_m  平均横向误差
max_abs_lateral_error_m   最大横向偏差
zero_speed_ratio_pct      速度为零比例
centerline_loss_ratio_pct 中心线丢失率
confidence_mean           平均置信度
safety_margin_mean_m      平均安全裕度
```

待完成事项：

- [ ] 确定实车横向误差真值获取方式。
- [ ] 确定碰株/擦碰记录标准。
- [ ] 确定人工接管判定标准。

## 7. 周期 E：完整行间闭环对照实验

### 7.1 前置条件

- [ ] 半闭环 3-5 m 连续稳定。
- [ ] 普通 PID 和质量感知 PID 都能安全停止。
- [ ] 感知低质量阶段不会导致车突然大角速度转向。
- [ ] 现场有 10-20 m 安全连续行间。

### 7.2 实验分组

推荐分组：

| 组别 | 控制方式 | 参数 |
|---|---|---|
| baseline_pid | 普通 PID | `use_quality_aware_control=false` |
| quality_aware_pid | 质量感知 PID | `use_quality_aware_control=true` |

如果后续增加质量感知纯追踪，可扩展：

| 组别 | 控制方式 |
|---|---|
| baseline_pure_pursuit | 普通 Pure Pursuit |
| quality_aware_pure_pursuit | 质量感知 Pure Pursuit |

当前项目中 Pure Pursuit 尚未接入 `/corridor_confidence` 和 `/corridor_safety_margin`，因此暂不作为质量感知主对照。

### 7.3 实验场景

| 场景 | 每组次数 | 目的 |
|---|---:|---|
| 正常直行 | 10 | 基础通过能力 |
| 叶片遮挡 | 10 | 低质量感知鲁棒性 |
| 局部缺株 | 10 | 中心线短时丢失恢复 |
| 初始偏航 | 10 | 偏航收敛能力 |
| 多行干扰 | 10 | 内侧行选择鲁棒性 |

如果时间有限，最低版本：

```text
正常直行：每组 5 次
叶片遮挡：每组 5 次
初始偏航：每组 5 次
```

### 7.4 每次闭环测试流程

1. 设置试验组别和场景。
2. 机器人放到起点。
3. 检查中心线、边界、置信度。
4. 开始 rosbag。
5. 启动控制器。
6. 行驶到目标距离或触发停止。
7. 停止控制器。
8. 停止 rosbag。
9. 记录现场 notes。
10. 检查 rosbag 是否有效。

### 7.5 闭环停止条件

出现以下任一情况立即停止：

```text
即将碰撞作物或人员
机器人偏离行间明显
中心线持续丢失
速度指令异常
TF 或 odom 中断
人工判断存在风险
```

### 7.6 不建议当前测试的内容

第一轮实地闭环不建议测试：

1. 地头 U 形换行。
2. 高速行驶。
3. 无人看守自主行驶。
4. 夜间或雨后复杂地形。
5. 行距过窄场景。

## 8. 周期 F：数据分析与论文整理

### 8.1 数据整理

建议每个 trial 输出：

```text
samples.csv
summary.csv
trajectory.png
metrics_timeseries.png
rviz_snapshot.png
notes.md
```

汇总输出：

```text
aggregate_summary.csv
comparison_table.png
comparison_bars.png
field_experiment_report.md
```

### 8.2 论文核心表格

表 1：实地场景说明。

| 场景 | 行距 | 作物高度 | 行长 | 地面 | 说明 |
|---|---:|---:|---:|---|---|

表 2：真实点云感知验证结果。

| 场景 | 有效率 | 中心线 RMSE | 行向角抖动 | 平均置信度 | 平均安全裕度 |
|---|---:|---:|---:|---:|---:|

表 3：闭环控制对照结果。

| 方法 | 成功率 | 横向 RMSE | 最大偏差 | 停车比例 | 人工接管 | 碰株次数 |
|---|---:|---:|---:|---:|---:|---:|

表 4：消融实验结果。

| 方法 | 有效率 | 中心线 RMSE | 低置信比例 | 抖动 | 处理时间 |
|---|---:|---:|---:|---:|---:|

### 8.3 论文核心图

建议准备：

1. 系统框图。
2. 实车平台与传感器安装图。
3. 真实冠下点云与中心线/边界叠加图。
4. 不同场景下中心线检测效果图。
5. 置信度、安全裕度时间序列。
6. 普通 PID 与质量感知 PID 轨迹对比。
7. 停车比例、安全裕度、成功率柱状图。
8. 消融实验柱状图。

### 8.4 统计方法

每组建议至少 5 次重复，最好 10 次。

报告：

```text
mean ± std
95% confidence interval
paired difference when trials are matched
success/failure count
```

样本量较小时，不要过度声称显著性。更稳妥表述：

```text
质量感知控制在保持相近横向跟踪误差的同时，降低了低质量感知阶段的停顿比例，并提高了平均安全裕度和连续通行能力。
```

## 9. 每个周期的产出与还需完成事项

### 周期 A 产出

```text
实车话题检查记录
TF 检查截图或 frames.pdf
雷达点云 RViz 截图
急停/遥控接管测试记录
```

还需完成：

- [ ] 统一实车启动 launch。
- [ ] 将雷达话题参数化，减少 remap 依赖。
- [ ] 整理现场录包脚本。

### 周期 B 产出

```text
真实玉米地 rosbag
每条 trial 的 notes.md
现场照片和视频
```

还需完成：

- [ ] 每类场景至少 3-5 条有效 rosbag。
- [ ] 确保每条包都包含原始点云、TF、odom。
- [ ] 尽量同时录算法输出。

### 周期 C 产出

```text
真实点云感知验证 summary.csv
诊断时间序列图
中心线/边界 RViz 截图
人工标注误差表
```

还需完成：

- [ ] 标注部分真值。
- [ ] 加入基线算法对照。
- [ ] 加入消融实验。

### 周期 D 产出

```text
3-5 m 半闭环 rosbag
成功/失败记录
人工接管记录
碰株/擦碰记录
```

还需完成：

- [ ] 确认自动控制低速稳定。
- [ ] 调整安全裕度阈值。
- [ ] 确认异常时能可靠停车。

### 周期 E 产出

```text
10-20 m 闭环对照 rosbag
普通 PID 与质量感知 PID 对照表
轨迹图
速度/置信度/安全裕度时间序列
```

还需完成：

- [ ] 每组每场景至少 5 次。
- [ ] 补充更多扰动场景。
- [ ] 若需要，扩展质量感知 Pure Pursuit。

### 周期 F 产出

```text
论文实验章节
实验表格
论文图片
统计分析结果
补充材料和视频
```

还需完成：

- [ ] 清理代码中的旧版本和 copy 文件。
- [ ] 固化实验参数 YAML。
- [ ] 整理 README 和复现实验说明。

## 10. 风险与处理

### 10.1 点云方向错误

表现：

```text
左行点云跑到右边
中心线明显偏到作物外侧
边界方向与车体运动方向不一致
```

处理：

1. 检查雷达 frame_id。
2. 检查雷达到 `base_link` 外参。
3. 检查 `base_link` 坐标定义。
4. 在 RViz 中显示 TF 坐标轴。

### 10.2 中心线频繁丢失

处理：

1. 放宽 ROI：`x_max`、`y_min`、`y_max`。
2. 调整 `z_min`、`z_max`，避免过滤掉作物点。
3. 调整 `min_cluster_size`。
4. 检查点云频率和点云密度。
5. 降低车速。

### 10.3 置信度长期过低

处理：

1. 检查 `desired_row_separation` 是否与真实行距一致。
2. 检查 `platform_width` 是否为真实车宽。
3. 调整 `min_row_separation` 和 `max_row_separation`。
4. 对真实数据重新标定置信度权重。

### 10.4 闭环转向过猛

处理：

1. 降低 `max_angular_speed`。
2. 降低 PID 增益。
3. 提高 `target_distance`。
4. 降低 `max_linear_speed`。
5. 先回到半闭环短距离测试。

### 10.5 多个节点抢占 `/cmd_vel`

处理：

1. 人工遥控采包时不要启动 `cornfield_navigation_node`。
2. 闭环测试时确认遥控接管逻辑优先级。
3. 使用 `ros2 topic info /cmd_vel` 检查发布者数量。

## 11. 推荐第一轮实地最小可行计划

如果只有一次实地机会，建议按以下最小计划执行：

### 上午

1. 设备安装与 TF 检查。
2. RViz 确认点云方向。
3. 正常直行人工遥控采包 5 次。
4. 叶片遮挡场景人工遥控采包 3 次。
5. 左右初始偏航人工遥控采包各 2 次。

### 中午

1. 快速回放 rosbag。
2. 检查是否包含点云、TF、odom。
3. 检查中心线和边界是否可用。

### 下午

1. 3-5 m 质量感知 PID 半闭环 3 次。
2. 若稳定，普通 PID 3 次。
3. 若仍稳定，质量感知 PID 与普通 PID 各补到 5 次。
4. 拍摄现场照片和视频。

### 当天晚上

1. 备份全部 rosbag。
2. 生成 bag info。
3. 整理 notes。
4. 初步统计 `/centerline_detection_diagnostics`。
5. 标记需要补采或参数调整的问题。

## 12. 当前项目后续工程建议

为了让实地验证更顺畅，建议后续补充：

1. 新增统一实车 launch 文件，包含雷达、TF、中心线检测和可选控制器。
2. 将 `/mid360_PointCloud2`、`base_frame`、`output_frame`、ROI、行距等整理成 YAML。
3. 新增现场录包脚本，例如 `scripts/record_field_perception_trial.sh`。
4. 新增真实点云离线重跑脚本，例如 `scripts/replay_field_perception_trial.sh`。
5. 新增人工标注与算法输出对齐脚本。
6. 清理 `centerline_extraction` 中旧版和 copy 文件，固定论文主算法版本。
7. 给 Pure Pursuit 增加质量感知接口，作为后续更完整控制对照。

