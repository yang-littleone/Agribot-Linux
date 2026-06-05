# 简单四轮差速驱动测试模型

## 概述

这是一个使用简单几何体（长方体+圆柱体）构建的四轮差速驱动机器人测试模型，用于验证Gazebo仿真中的差速驱动配置。

## 模型特点

- **底盘**: 蓝色长方体 (0.3m × 0.2m × 0.1m)
- **轮子**: 4个白色圆柱体 (半径0.05m, 厚度0.03m)
- **轮距**: 0.2m
- **所有轮轴方向**: `xyz="0 0 1"` (Z轴)
- **质量**: 底盘5kg, 每个轮子0.5kg

## 关键设计

### 轮轴方向
在这个模型中，**所有4个轮子的joint轴方向都是 `xyz="0 0 1"`（Z轴正方向）**，这是通过设置 `rpy="${-pi/2} 0 0"` 将圆柱体旋转90度实现的。

```xml
<joint name="lf_wheel_joint" type="continuous">
    <origin xyz="..." rpy="${-pi/2} 0 0"/>
    <axis xyz="0 0 1"/>
</joint>
```

这意味着左右轮的旋转轴方向是**相同**的。

### 控制器配置

在 `simple_4wd_controller.yaml` 中：
```yaml
right_wheel_radius_multiplier: 1.0  # 保持为1.0
```

## 测试步骤

### 1. 启动仿真

```bash
ros2 launch agribot_simulation simple_4wd_test.launch.py
```

### 2. 启动键盘控制

在新的终端中：
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### 3. 测试运动

- **前进**: 按 `i` 键
- **后退**: 按 `,` 键
- **逆时针旋转**: 按 `j` 键
- **顺时针旋转**: 按 `l` 键
- **停止**: 按空格键

### 4. 或使用命令行测试

```bash
# 逆时针旋转
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.0}}'

# 顺时针旋转
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -1.0}}'

# 前进
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

## 预期结果

✅ **如果配置正确**：
- 机器人应该能够流畅地前进、后退
- 机器人应该能够流畅地原地旋转（逆时针和顺时针）
- 旋转时左右轮应该反向转动

❌ **如果不能旋转**：
可能的原因：
1. 轮轴方向配置问题
2. 控制器参数问题
3. Gazebo物理引擎问题

## 调试方法

### 1. 检查控制器是否加载

```bash
ros2 control list_controllers
```

应该看到：
- `simple_4wd_joint_state_broadcaster` - active
- `simple_4wd_diff_drive_controller` - active

### 2. 检查话题

```bash
ros2 topic list
```

应该看到：
- `/cmd_vel`
- `/odom`
- `/joint_states`

### 3. 查看轮子速度

```bash
ros2 topic echo /joint_states
```

当发送旋转命令时，应该看到左右轮的速度值相反。

### 4. 检查TF树

```bash
ros2 run tf2_tools view_frames.py
evince frames.pdf
```

## 与AgriBot模型的对比

| 特性 | Simple 4WD Test | AgriBot |
|------|----------------|---------|
| 几何体 | 简单几何体 | STL网格 |
| 轮轴方向 | `xyz="0 0 1"` (Z轴) | `xyz="0 1 0"` (Y轴) |
| 轮子安装 | 通过rpy旋转 | 直接安装 |
| right_wheel_radius_multiplier | 1.0 | 1.0 |

## 重要说明

### 为什么这个模型可以工作？

虽然所有轮轴方向相同，但关键在于：

1. **轮子的安装方式**：通过 `rpy="${-pi/2} 0 0"` 将圆柱体旋转90度，使得圆柱体的轴线从Z轴变为Y轴方向
2. **Joint轴定义**：`<axis xyz="0 0 1"/>` 定义的是关节的旋转轴，在旋转后的坐标系中，这个轴对应于轮子的实际旋转方向
3. **左右轮对称性**：由于使用了相同的安装方式，左右轮的旋转方向在物理上已经是相反的（一个朝前转，一个朝后转）

### 如果Simple模型能旋转但AgriBot不能

这说明问题在于：
1. AgriBot的URDF中轮子的几何定义或安装方式
2. AgriBot的mesh文件可能有问题
3. AgriBot的物理参数（摩擦系数、质量等）可能需要调整

## 文件位置

- URDF: `/src/agribot_simulation/urdf/simple_4wd_test.urdf`
- 控制器配置: `/src/agribot_simulation/config/simple_4wd_controller.yaml`
- Launch文件: `/src/agribot_simulation/launch/simple_4wd_test.launch.py`

## 下一步

1. 先测试这个简单模型，确认它能正常旋转
2. 如果能旋转，对比两个模型的差异
3. 根据差异调整AgriBot的配置
