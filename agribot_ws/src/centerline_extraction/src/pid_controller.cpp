#include "centerline_extraction/pid_controller.hpp"
#include <algorithm>
#include <cmath>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>

PIDController::PIDController() : Node("pid_controller"), has_center_line_(false), first_run_(true)
{
    // 订阅玉米行中心线（路径）
    center_line_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/corn_row_center_line", 10,
        std::bind(&PIDController::center_line_callback, this, std::placeholders::_1));

    // 订阅里程计信息（小车位姿）
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&PIDController::odom_callback, this, std::placeholders::_1));

    confidence_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "/corridor_confidence", 10,
        std::bind(&PIDController::confidence_callback, this, std::placeholders::_1));

    safety_margin_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "/corridor_safety_margin", 10,
        std::bind(&PIDController::safety_margin_callback, this, std::placeholders::_1));

    // 发布速度控制指令
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10);

    // 发布目标点可视化标记（调试用）
    target_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "/target_point", 10);

    // 声明并初始化参数
    this->declare_parameter("target_distance", 0.4);   // 目标跟随距离（米）
    this->declare_parameter("max_linear_speed", 0.4);  // 最大线速度（米/秒）
    this->declare_parameter("min_linear_speed", 0.1);  // 最小线速度（米/秒）
    this->declare_parameter("max_angular_speed", 1.0); // 最大角速度（弧度/秒）
    this->declare_parameter("confidence_high_threshold", 0.75);
    this->declare_parameter("confidence_low_threshold", 0.45);
    this->declare_parameter("confidence_stop_threshold", 0.25);
    this->declare_parameter("confidence_min_speed_factor", 0.2);
    this->declare_parameter("safety_margin_high", 0.20);
    this->declare_parameter("safety_margin_mid", 0.10);
    this->declare_parameter("safety_margin_stop", 0.05);
    this->declare_parameter("max_low_confidence_frames", 10);
    this->declare_parameter("use_quality_aware_control", true);

    // 横向PID参数
    this->declare_parameter("lateral_kp", 20.0); // 比例系数
    this->declare_parameter("lateral_ki", 1.0); // 积分系数
    this->declare_parameter("lateral_kd", 10.0); // 微分系数

    // 航向PID参数
    this->declare_parameter("heading_kp", 20.0); // 比例系数
    this->declare_parameter("heading_ki", 1.0); // 积分系数
    this->declare_parameter("heading_kd", 10.0); // 微分系数

    this->declare_parameter("debug_mode", false); // 调试模式开关

    // 读取参数
    get_parameters();

    // 初始化状态变量
    current_x_ = 0.0;
    current_y_ = 0.0;
    current_yaw_ = 0.0;
    current_linear_vel_ = 0.0;
    current_angular_vel_ = 0.0;
    has_last_valid_center_line_ = false;
    use_recovery_path_ = false;
    low_confidence_count_ = 0;
    corridor_confidence_ = 1.0;
    corridor_safety_margin_ = safety_margin_high_;
    has_corridor_confidence_ = false;
    has_corridor_safety_margin_ = false;

    // 初始化PID控制器状态变量
    lateral_integral_ = 0.0;
    lateral_previous_error_ = 0.0;
    heading_integral_ = 0.0;
    heading_previous_error_ = 0.0;

    RCLCPP_INFO(this->get_logger(), "PID控制器初始化完成（目标距离: %.2f米）", target_distance_);
    RCLCPP_INFO(this->get_logger(), "置信度/安全裕度控制: %s",
                use_quality_aware_control_ ? "启用" : "关闭");
}

void PIDController::get_parameters()
{
    this->get_parameter("target_distance", target_distance_);
    this->get_parameter("max_linear_speed", max_linear_speed_);
    this->get_parameter("min_linear_speed", min_linear_speed_);
    this->get_parameter("max_angular_speed", max_angular_speed_);
    this->get_parameter("confidence_high_threshold", confidence_high_threshold_);
    this->get_parameter("confidence_low_threshold", confidence_low_threshold_);
    this->get_parameter("confidence_stop_threshold", confidence_stop_threshold_);
    this->get_parameter("confidence_min_speed_factor", confidence_min_speed_factor_);
    this->get_parameter("safety_margin_high", safety_margin_high_);
    this->get_parameter("safety_margin_mid", safety_margin_mid_);
    this->get_parameter("safety_margin_stop", safety_margin_stop_);
    this->get_parameter("max_low_confidence_frames", max_low_confidence_frames_);
    this->get_parameter("use_quality_aware_control", use_quality_aware_control_);

    this->get_parameter("lateral_kp", lateral_kp_);
    this->get_parameter("lateral_ki", lateral_ki_);
    this->get_parameter("lateral_kd", lateral_kd_);

    this->get_parameter("heading_kp", heading_kp_);
    this->get_parameter("heading_ki", heading_ki_);
    this->get_parameter("heading_kd", heading_kd_);

    this->get_parameter("debug_mode", debug_mode_);
}

void PIDController::center_line_callback(const nav_msgs::msg::Path::SharedPtr msg)
{
    if (msg->poses.empty())
    {
        RCLCPP_WARN(this->get_logger(), "收到空的中心线，当前帧不更新中心线");
        has_center_line_ = false;
        return;
    }

    center_line_ = *msg;
    has_center_line_ = true;
    if (!use_quality_aware_control_ ||
        !has_corridor_confidence_ ||
        corridor_confidence_ >= confidence_low_threshold_)
    {
        last_valid_center_line_ = center_line_;
        has_last_valid_center_line_ = true;
        low_confidence_count_ = 0;
    }
    RCLCPP_DEBUG(this->get_logger(), "收到中心线（%zu个点）", center_line_.poses.size());
}

void PIDController::confidence_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    corridor_confidence_ = std::clamp(static_cast<double>(msg->data), 0.0, 1.0);
    has_corridor_confidence_ = true;
}

void PIDController::safety_margin_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    corridor_safety_margin_ = static_cast<double>(msg->data);
    has_corridor_safety_margin_ = true;
}

void PIDController::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // 更新当前位姿
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;
    current_linear_vel_ = msg->twist.twist.linear.x;
    current_angular_vel_ = msg->twist.twist.angular.z;

    // 从四元数解析偏航角（yaw）
    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, current_yaw_);

    if (should_stop_for_safety())
    {
        geometry_msgs::msg::Twist stop_cmd;
        cmd_vel_pub_->publish(stop_cmd);
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "安全裕度不足 %.3f m，停车", corridor_safety_margin_);
        return;
    }

    const bool confidence_low = use_quality_aware_control_ &&
                                has_corridor_confidence_ &&
                                corridor_confidence_ < confidence_low_threshold_;
    use_recovery_path_ = false;

    if (confidence_low)
    {
        low_confidence_count_++;
        if (has_last_valid_center_line_ &&
            low_confidence_count_ <= max_low_confidence_frames_ &&
            corridor_confidence_ >= confidence_stop_threshold_)
        {
            center_line_ = last_valid_center_line_;
            has_center_line_ = true;
            use_recovery_path_ = true;
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "低置信度 %.2f，使用历史中心线恢复跟踪 (%d/%d)",
                                 corridor_confidence_, low_confidence_count_, max_low_confidence_frames_);
        }
        else
        {
            geometry_msgs::msg::Twist stop_cmd;
            cmd_vel_pub_->publish(stop_cmd);
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "中心线置信度过低 %.2f，停车", corridor_confidence_);
            return;
        }
    }
    else
    {
        low_confidence_count_ = 0;
    }

    if (has_center_line_ && !center_line_.poses.empty())
    {
        auto cmd_vel = calculate_control_command();
        cmd_vel_pub_->publish(cmd_vel);
    }
    else
    {
        // 当没有中心线时，发布零速度指令让小车停止
        geometry_msgs::msg::Twist stop_cmd;
        stop_cmd.linear.x = 0.0;
        stop_cmd.angular.z = 0.0;
        cmd_vel_pub_->publish(stop_cmd);
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "未收到中心线，发布停止指令");
    }
}

geometry_msgs::msg::PointStamped PIDController::find_target_point()
{
    geometry_msgs::msg::PointStamped target_point;
    target_point.header.frame_id = center_line_.header.frame_id;
    target_point.header.stamp = this->now();

    // 1. 找到路径上离小车最近的点
    size_t closest_idx = 0;
    double min_dist = 1e9;
    for (size_t i = 0; i < center_line_.poses.size(); ++i)
    {
        const auto &p = center_line_.poses[i].pose.position;
        double dx = p.x - current_x_;
        double dy = p.y - current_y_;
        double dist = std::hypot(dx, dy);
        if (dist < min_dist)
        {
            min_dist = dist;
            closest_idx = i;
        }
    }

    // 2. 从最近点开始，寻找距离小车目标距离的点
    double accumulated_dist = 0.0;
    for (size_t i = closest_idx; i < center_line_.poses.size() - 1; ++i)
    {
        const auto &p1 = center_line_.poses[i].pose.position;
        const auto &p2 = center_line_.poses[i + 1].pose.position;

        // 计算路径段的长度
        double segment_length = std::hypot(p2.x - p1.x, p2.y - p1.y);

        // 如果加上这一段超过了目标距离，则在该段上插值
        if (accumulated_dist + segment_length >= target_distance_)
        {
            double remaining_dist = target_distance_ - accumulated_dist;
            double t = remaining_dist / segment_length;

            target_point.point.x = p1.x + t * (p2.x - p1.x);
            target_point.point.y = p1.y + t * (p2.y - p1.y);
            target_point.point.z = 0.0;

            return target_point;
        }

        accumulated_dist += segment_length;
    }

    // 如果路径不够长，取最后一个点
    // target_point.point = center_line_.poses.back().pose.position;

    // 原逻辑：target_point.point = center_line_.poses.back().pose.position;
    // 新逻辑：若最后一个点落后于小车，强制设为小车前方0.4米（避免目标点在小车后方）
    auto last_point = center_line_.poses.back().pose.position;
    if (last_point.x < current_x_)
    {                                                         // 若中心线最后一个点在小车后方
        target_point.point.x = current_x_ + target_distance_; // 强制设为前方目标距离米
        target_point.point.y = current_y_;                    // 临时用小车y坐标（后续会被中心线更新覆盖）
    }
    else
    {
        target_point.point = last_point; // 正常取最后一个点
    }

    return target_point;
}

double PIDController::compute_pid(double error, double dt, double &integral, double &previous_error,
                                  double kp, double ki, double kd)
{
    // 防止除零错误和时间间隔过大
    if (dt <= 0.0 || dt > 1.0)
    {
        dt = 0.05;
    }

    // 积分项
    integral += error * dt;

    // 微分项
    double derivative = (error - previous_error) / dt;

    // PID输出
    double output = kp * error + ki * integral + kd * derivative;

    // 更新previous_error
    previous_error = error;

    return output;
}

double PIDController::compute_confidence_factor() const
{
    if (!use_quality_aware_control_)
    {
        return 1.0;
    }

    if (!has_corridor_confidence_)
    {
        return 1.0;
    }

    if (use_recovery_path_)
    {
        return confidence_min_speed_factor_;
    }

    return std::clamp(corridor_confidence_, confidence_min_speed_factor_, 1.0);
}

double PIDController::compute_safety_factor() const
{
    if (!use_quality_aware_control_)
    {
        return 1.0;
    }

    if (!has_corridor_safety_margin_)
    {
        return 1.0;
    }

    if (corridor_safety_margin_ < safety_margin_stop_)
    {
        return 0.0;
    }
    if (corridor_safety_margin_ < safety_margin_mid_)
    {
        return 0.3;
    }
    if (corridor_safety_margin_ < safety_margin_high_)
    {
        return 0.6;
    }
    return 1.0;
}

bool PIDController::should_stop_for_safety() const
{
    return use_quality_aware_control_ &&
           has_corridor_safety_margin_ &&
           corridor_safety_margin_ < safety_margin_stop_;
}

void PIDController::publish_target_marker(const geometry_msgs::msg::PointStamped &point)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "odom"; // 统一使用 odom 坐标系以确保一致性
    marker.header.stamp = this->now();
    marker.ns = "target";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position = point.point;
    marker.pose.position.z = 0.2; // 稍微抬高一点，便于在地面上看到
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker.lifetime = rclcpp::Duration::from_seconds(0.2);
    target_marker_pub_->publish(marker);
}

geometry_msgs::msg::Twist PIDController::calculate_control_command()
{
    geometry_msgs::msg::Twist cmd_vel;

    // 调试模式：输出固定速度测试转向
    if (debug_mode_)
    {
        cmd_vel.linear.x = 0.1;
        cmd_vel.angular.z = 0.0;
        RCLCPP_INFO(this->get_logger(), "调试模式：线速度=0.1m/s, 角速度=0");
        return cmd_vel;
    }

    // 1. 寻找目标点（中心线前方target_distance_米处的点）
    target_point_ = find_target_point();
    publish_target_marker(target_point_); // 可视化目标点

    // 2. 计算时间差
    rclcpp::Time current_time = this->now();
    double dt = 0.0;

    if (!first_run_)
    {
        dt = (current_time - last_time_).seconds();
    }
    else
    {
        dt = 0.05; // 假设50ms
        first_run_ = false;
    }
    last_time_ = current_time;

    // 3. 将目标点转换到小车坐标系
    double dx = target_point_.point.x - current_x_;
    double dy = target_point_.point.y - current_y_;

    // 小车坐标系下的目标点位置
    double x_veh = dx * cos(current_yaw_) + dy * sin(current_yaw_);  // 前向距离
    double y_veh = -dx * sin(current_yaw_) + dy * cos(current_yaw_); // 横向距离（左正右负）

    // 4. 计算横向误差（y_veh就是我们需要消除的横向误差）
    double lateral_error = y_veh;

    // 5. 计算航向误差（目标点相对于小车朝向的角度）
    double heading_error = atan2(y_veh, x_veh);

    // 6. 使用PID控制器计算控制输出
    // 横向控制（主要影响转向）
    double lateral_control = compute_pid(lateral_error, dt, lateral_integral_, lateral_previous_error_,
                                         lateral_kp_, lateral_ki_, lateral_kd_);

    // 航向控制（辅助调整转向）
    double heading_control = compute_pid(heading_error, dt, heading_integral_, heading_previous_error_,
                                         heading_kp_, heading_ki_, heading_kd_);

    const double confidence_factor = compute_confidence_factor();
    const double safety_factor = compute_safety_factor();
    if (safety_factor <= 0.0)
    {
        return cmd_vel;
    }

    // 7. 组合控制输出
    double angular_vel = lateral_control + heading_control;
    const double current_max_angular_speed = std::max(0.1, max_angular_speed_ * safety_factor);
    angular_vel = std::clamp(angular_vel, -current_max_angular_speed, current_max_angular_speed);

    // 8. 计算线速度：中心线质量、安全裕度和转向幅度共同调速
    const double turning_factor = std::clamp(
        1.0 - 0.5 * std::abs(angular_vel) / std::max(current_max_angular_speed, 1e-3),
        0.2,
        1.0);
    double linear_speed = max_linear_speed_ * confidence_factor * safety_factor * turning_factor;
    if (linear_speed > 1e-6)
    {
        const double quality_min_speed = min_linear_speed_ * std::min(confidence_factor, safety_factor);
        linear_speed = std::max(linear_speed, quality_min_speed);
    }

    // 9. 赋值控制指令
    cmd_vel.linear.x = linear_speed;
    cmd_vel.angular.z = angular_vel;

    // 调试日志
    RCLCPP_DEBUG(this->get_logger(),
                 "目标距离: %.2f, 横向误差: %.2f, 航向误差: %.2f, 线速度: %.2f, 角速度: %.2f, 置信因子: %.2f, 安全因子: %.2f",
                 target_distance_, lateral_error, heading_error, linear_speed, angular_vel,
                 confidence_factor, safety_factor);

    return cmd_vel;
}
