# 同一 world 五次测试执行方案

本文档用于指导当前算法在同一个 Gazebo world 中进行 5 次重复测试。当前测试目标不是马上做完整论文实验，而是先验证：

1. 中心线提取是否连续稳定；
2. `corridor_confidence` 是否能反映检测质量；
3. `corridor_safety_margin` 是否能影响 PID 控制速度；
4. 新 PID 是否会在低置信度或低安全裕度时主动降速/停车；
5. 五次重复测试结果是否具有一致性。

## 1. 当前测试链路

当前用户实际测试链路为：

```bash
ros2 launch diff_drive_robot robot.launch.py world:=yumidiliuhang.world
ros2 run centerline_extraction corn_row_detector_projection
ros2 run centerline_extraction cornfield_navigation_node
```

对应数据流：

```text
Gazebo world
  ↓
/mid360_PointCloud2
  ↓
corn_row_detector_projection
  ↓
/corn_row_center_line
/corridor_confidence
/corridor_safety_margin
/corridor_width
  ↓
cornfield_navigation_node 中的 PIDController
  ↓
/cmd_vel
  ↓
Gazebo 差速驱动
```

## 2. 测试前准备

进入工作空间：

```bash
cd /home/xkai/agribot/agribot_ws
source install/setup.bash
```

建议先重新编译一次，确认最新修改生效：

```bash
colcon build --packages-select centerline_extraction
source install/setup.bash
```

创建测试数据目录：

```bash
mkdir -p ~/agribot_test_logs/yumidiliuhang_quality_pid
```

## 3. 每次测试需要记录的话题

建议记录以下 topic：

```text
/clock
/tf
/tf_static
/odom
/cmd_vel
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

其中最重要的是：

```text
/odom
/cmd_vel
/corn_row_center_line
/corridor_confidence
/corridor_safety_margin
/corridor_width
```

## 4. 推荐终端布局

建议使用 4 个终端。

### 终端 1：启动 Gazebo

```bash
cd /home/xkai/agribot/agribot_ws
source install/setup.bash
ros2 launch diff_drive_robot robot.launch.py world:=yumidiliuhang.world
```

等待 Gazebo 完全启动，机器人和玉米行模型加载完成。

### 终端 2：启动中心线提取

```bash
cd /home/xkai/agribot/agribot_ws
source install/setup.bash
ros2 run centerline_extraction corn_row_detector_projection --ros-args -p use_sim_time:=true
```

### 终端 3：启动路径跟踪控制

默认 `main.cpp` 中 `controller_type = pid`，因此直接运行即可：

```bash
cd /home/xkai/agribot/agribot_ws
source install/setup.bash
ros2 run centerline_extraction cornfield_navigation_node --ros-args -p use_sim_time:=true
```

如果后续想强制指定 PID，可用：

```bash
ros2 run centerline_extraction cornfield_navigation_node --controller_type pid --ros-args -p use_sim_time:=true
```

### 终端 4：记录 rosbag

每次测试单独记录一个 bag。比如第 1 次：

```bash
cd /home/xkai/agribot/agribot_ws
source install/setup.bash

ros2 bag record \
  -o ~/agribot_test_logs/yumidiliuhang_quality_pid/trial_01 \
  /clock \
  /tf \
  /tf_static \
  /odom \
  /cmd_vel \
  /corn_row_center_line \
  /corn_row_center_line_viz \
  /under_canopy_left_boundary \
  /under_canopy_right_boundary \
  /corridor_width \
  /corridor_safety_margin \
  /corridor_confidence \
  /point_cloud_projected \
  /left_row_points \
  /right_row_points
```

第 2-5 次只需要把输出目录改成：

```text
trial_02
trial_03
trial_04
trial_05
```

## 5. 五次测试执行流程

每一次测试按以下步骤执行。

### Step 1：重启仿真

为了保证五次测试初始条件尽量一致，建议每次测试都完整重启：

1. 关闭 Gazebo；
2. 关闭中心线提取节点；
3. 关闭路径跟踪节点；
4. 关闭 rosbag record；
5. 重新按终端 1-4 顺序启动。

不要只在 Gazebo 中手动拖动车，因为那样机器人速度、TF、控制器积分状态可能没有完全重置。

### Step 2：确认 topic 正常

在任意新终端检查：

```bash
ros2 topic list | grep corridor
```

应能看到：

```text
/corridor_confidence
/corridor_safety_margin
/corridor_width
```

检查中心线：

```bash
ros2 topic echo /corridor_confidence --once
ros2 topic echo /corridor_safety_margin --once
ros2 topic echo /corn_row_center_line --once
```

### Step 3：开始记录 rosbag

先启动 rosbag record，再启动路径跟踪，或者至少保证机器人开始移动前 rosbag 已经在记录。

推荐顺序：

```text
Gazebo
中心线提取
rosbag record
路径跟踪
```

### Step 4：测试持续时间

每次测试建议固定时长：

```text
60 s
```

如果 world 较短，机器人提前走完，则记录实际完成时间。

如果出现严重跑偏、碰撞或停车，也记录原因，不要直接覆盖该次数据。

### Step 5：结束记录

在 rosbag 终端按：

```text
Ctrl + C
```

然后依次关闭路径跟踪、中心线提取、Gazebo。

## 6. 五次测试命名规范

建议命名如下：

```text
trial_01
trial_02
trial_03
trial_04
trial_05
```

对应目录：

```bash
~/agribot_test_logs/yumidiliuhang_quality_pid/trial_01
~/agribot_test_logs/yumidiliuhang_quality_pid/trial_02
~/agribot_test_logs/yumidiliuhang_quality_pid/trial_03
~/agribot_test_logs/yumidiliuhang_quality_pid/trial_04
~/agribot_test_logs/yumidiliuhang_quality_pid/trial_05
```

## 7. 每次测试人工记录表

建议额外手动记录一个表格，文件可命名为：

```bash
~/agribot_test_logs/yumidiliuhang_quality_pid/manual_notes.md
```

内容模板：

```markdown
# yumidiliuhang quality PID five-run notes

| Trial | 是否成功通过 | 是否停车 | 是否明显跑偏 | 是否碰撞 | 线是否消失 | 平均观感 | 备注 |
|---|---|---|---|---|---|---|---|
| 01 |  |  |  |  |  |  |  |
| 02 |  |  |  |  |  |  |  |
| 03 |  |  |  |  |  |  |  |
| 04 |  |  |  |  |  |  |  |
| 05 |  |  |  |  |  |  |  |
```

重点记录：

```text
1. 是否出现中心线突然大幅跳动；
2. 置信度降低时小车是否明显减速；
3. 安全裕度很小时是否减速或停车；
4. 是否出现无意义急转；
5. 是否能稳定走完整行。
```

## 8. 测试中建议打开的 RViz 显示项

Fixed Frame 建议设为：

```text
odom
```

确保 RViz 使用仿真时间：

```bash
ros2 param set /rviz use_sim_time true
```

建议显示：

```text
RobotModel
TF
/point_cloud_projected
/left_row_points
/right_row_points
/corn_row_center_line
/corn_row_center_line_viz
/under_canopy_left_boundary
/under_canopy_right_boundary
/target_point
```

## 9. 如何初步判断改动是否有效

### 9.1 看线速度是否会响应置信度

测试中另开终端：

```bash
ros2 topic echo /corridor_confidence
```

再开一个终端：

```bash
ros2 topic echo /cmd_vel
```

观察：

```text
当 corridor_confidence 下降时，cmd_vel.linear.x 是否下降。
```

如果 confidence 明显低，但速度完全不变，则说明质量控制没有发挥作用。

### 9.2 看安全裕度是否会影响速度和角速度

观察：

```bash
ros2 topic echo /corridor_safety_margin
ros2 topic echo /cmd_vel
```

期望现象：

```text
safety_margin 大：正常速度
safety_margin 中等：速度降低
safety_margin 很小：速度明显降低或停车
```

### 9.3 看低置信度是否会短时保持历史路径

如果中心线偶尔闪烁或置信度短时降低，期望：

```text
小车不要立刻急停或急转；
应低速沿历史中心线继续一小段。
```

如果连续低置信度，期望：

```text
最终停车。
```

## 10. 五次测试结束后的最小分析

先不写复杂脚本时，可以用以下命令查看 bag 信息：

```bash
ros2 bag info ~/agribot_test_logs/yumidiliuhang_quality_pid/trial_01
```

检查每个 bag 是否包含：

```text
/odom
/cmd_vel
/corridor_confidence
/corridor_safety_margin
/corn_row_center_line
```

五次都检查：

```bash
for i in 01 02 03 04 05; do
  echo "===== trial_$i ====="
  ros2 bag info ~/agribot_test_logs/yumidiliuhang_quality_pid/trial_$i | grep -E "/odom|/cmd_vel|/corridor_confidence|/corridor_safety_margin|/corn_row_center_line"
done
```

## 11. 初步合格标准

五次测试中，如果满足以下条件，说明当前改动初步有效：

```text
1. 至少 4/5 次能稳定完成测试路线；
2. 没有出现明显急转撞向作物；
3. cmd_vel.linear.x 会随 confidence 或 safety_margin 降低而降低；
4. 短时中心线异常时不会立即失控；
5. 长时间低置信度或安全裕度不足时能停车。
```

如果出现以下情况，则需要继续调整：

```text
1. 速度一直很低，说明阈值太保守；
2. 经常停车，说明 confidence 或 safety_margin 计算/阈值需要调；
3. 低置信度时仍急转，说明历史路径保持逻辑还不够；
4. 安全裕度小但仍高速，说明 safety_factor 没发挥作用；
5. 控制器频繁在走/停之间抖动，说明阈值需要滞回或滤波。
```

## 12. 建议先不要做的事情

第一次五次测试先不要同时改很多参数。建议保持当前默认参数：

```text
confidence_high_threshold = 0.75
confidence_low_threshold = 0.45
confidence_stop_threshold = 0.25
confidence_min_speed_factor = 0.2
safety_margin_high = 0.20
safety_margin_mid = 0.10
safety_margin_stop = 0.05
max_low_confidence_frames = 10
```

如果五次测试后发现太保守，再逐个调整。

## 13. 后续正式论文实验建议

这次五次测试只是初步验证新控制策略是否工作。正式论文实验还需要：

```text
原始 PID vs 质量感知 PID
正常场景 vs 遮挡场景 vs 通道变窄场景
每组至少 5 次重复
统计平均横向误差、最大横向误差、最小安全距离、平均速度、成功率
```

后续建议给 PID 增加参数：

```text
use_corridor_quality_control
```

这样可以在同一份代码中切换：

```text
false：原始 PID
true：置信度与安全裕度感知 PID
```

这会更方便做论文对比实验。
