#!/bin/bash
# AgriBot 旋转测试脚本

echo "========================================="
echo "AgriBot Gazebo 旋转测试"
echo "========================================="
echo ""

# 检查是否已经启动了gazebo
if ! ros2 topic list | grep -q "/cmd_vel"; then
    echo "❌ 错误: 未检测到 /cmd_vel 话题"
    echo "请先启动 Gazebo 仿真:"
    echo "  ros2 launch agribot_simulation gazebo_sim.launch.py"
    exit 1
fi

echo "✅ 检测到 /cmd_vel 话题，Gazebo 正在运行"
echo ""
echo "开始测试..."
echo ""

# 测试1: 逆时针旋转
echo "🔄 测试1: 逆时针旋转 (3秒)"
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.0}}"
sleep 3

# 停止
echo "⏹ 停止"
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
sleep 1

# 测试2: 顺时针旋转
echo "🔄 测试2: 顺时针旋转 (3秒)"
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -1.0}}"
sleep 3

# 停止
echo "⏹ 停止"
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
sleep 1

# 测试3: 前进
echo "⬆ 测试3: 前进 (2秒)"
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
sleep 2

# 停止
echo "⏹ 停止"
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
sleep 1

# 测试4: 后退
echo "⬇ 测试4: 后退 (2秒)"
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: -0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
sleep 2

# 停止
echo "⏹ 停止"
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
sleep 1

# 测试5: 前进+旋转
echo "↗ 测试5: 前进+逆时针旋转 (3秒)"
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.8}}"
sleep 3

# 停止
echo "⏹ 停止"
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

echo ""
echo "========================================="
echo "✅ 测试完成！"
echo "========================================="
echo ""
echo "请观察 Gazebo 中的机器人运动："
echo "  - 旋转应该流畅且明显"
echo "  - 前进后退应该正常"
echo "  - 组合运动应该自然"
echo ""
echo "如果仍然不能旋转，请检查："
echo "  1. right_wheel_radius_multiplier 是否设置为 -1.0"
echo "  2. 控制器是否成功加载"
echo "  3. 查看终端是否有错误信息"
echo ""
