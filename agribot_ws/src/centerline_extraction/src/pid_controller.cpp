#include "centerline_extraction/pid_controller.hpp"
#include <cmath>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>

PIDController::PIDController() : Node("pid_controller"), has_center_line_(false),
    linear_error_sum_(0.0), angular_error_sum_(0.0),
    last_cross_track_error_(0.0), last_heading_error_(0.0)
{
    // 订阅玉米行中心线（路径）
    center_line_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/corn_row_center", 10,
        std::bind(&PIDController::center_line_callback, this, std::placeholders::_1));
    
    // 订阅里程计信息（小车位姿）
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&PIDController::odom_callback, this, std::placeholders::_1));
    
    // 发布速度控制指令
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10);
    
    // 发布目标点可视化标记（调试用）
    lookahead_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "/lookahead_point", 10);

    // 声明并初始化参数
    this->declare_parameter("kp_linear", 1.0);              // 线速度比例系数
    this->declare_parameter("ki_linear", 0.0);              // 线速度积分系数
    this->declare_parameter("kd_linear", 0.0);              // 线速度微分系数
    
    this->declare_parameter("kp_angular", 2.0);             // 角速度比例系数
    this->declare_parameter("ki_angular", 0.0);             // 角速度积分系数
    this->declare_parameter("kd_angular", 0.1);             // 角速度微分系数

    this->declare_parameter("max_linear_speed", 0.4);       // 最大线速度（米/秒）
    this->declare_parameter("min_linear_speed", 0.1);       // 最小线速度（米/秒）
    this->declare_parameter("max_angular_speed", 1.5);      // 最大角速度（弧度/秒）
    
    this->declare_parameter("debug_mode", false);           // 调试模式开关

    // 读取参数
    get_parameters();

    // 初始化状态变量
    current_x_ = 0.0;
    current_y_ = 0.0;
    current_yaw_ = 0.0;
    current_linear_vel_ = 0.0;
    
    last_time_ = this->now();

    RCLCPP_INFO(this->get_logger(), "PID控制器初始化完成");
}

void PIDController::get_parameters()
{
    this->get_parameter("kp_linear", kp_linear_);
    this->get_parameter("ki_linear", ki_linear_);
    this->get_parameter("kd_linear", kd_linear_);
    
    this->get_parameter("kp_angular", kp_angular_);
    this->get_parameter("ki_angular", ki_angular_);
    this->get_parameter("kd_angular", kd_angular_);

    this->get_parameter("max_linear_speed", max_linear_speed_);
    this->get_parameter("min_linear_speed", min_linear_speed_);
    this->get_parameter("max_angular_speed", max_angular_speed_);
    
    this->get_parameter("debug_mode", debug_mode_);
}

void PIDController::center_line_callback(const nav_msgs::msg::Path::SharedPtr msg)
{
    if (msg->poses.empty()) {
        RCLCPP_WARN(this->get_logger(), "收到空的中心线，忽略");
        return;
    }
    
    center_line_ = *msg;
    has_center_line_ = true;
    RCLCPP_DEBUG(this->get_logger(), "收到中心线（%zu个点）", center_line_.poses.size());
}

void PIDController::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // 更新当前位姿
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;

    // 从四元数解析偏航角（yaw）
    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, current_yaw_);

    // 有中心线时计算控制指令
    if (has_center_line_ && !center_line_.poses.empty()) {
        auto cmd_vel = calculate_control_command();
        cmd_vel_pub_->publish(cmd_vel);
    } else if (!has_center_line_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "未收到中心线，不发布控制指令");
    }
}

geometry_msgs::msg::PointStamped PIDController::find_closest_point()
{
    geometry_msgs::msg::PointStamped closest_point;
    closest_point.header.frame_id = center_line_.header.frame_id;
    closest_point.header.stamp = this->now();

    // 找到路径上离小车最近的点
    size_t closest_idx = 0;
    double min_dist = 1e9;
    for (size_t i = 0; i < center_line_.poses.size(); ++i) {
        const auto& p = center_line_.poses[i].pose.position;
        double dx = p.x - current_x_;
        double dy = p.y - current_y_;
        double dist = std::hypot(dx, dy);
        if (dist < min_dist) {
            min_dist = dist;
            closest_idx = i;
        }
    }
    
    closest_point.point = center_line_.poses[closest_idx].pose.position;
    return closest_point;
}

double PIDController::calculate_cross_track_error(const geometry_msgs::msg::PointStamped& closest_point)
{
    // 计算横向误差（即到路径的垂直距离）
    double dx = closest_point.point.x - current_x_;
    double dy = closest_point.point.y - current_y_;
    
    // 横向误差 = -sin(yaw) * dx + cos(yaw) * dy
    return -sin(current_yaw_) * dx + cos(current_yaw_) * dy;
}

double PIDController::calculate_heading_error(const geometry_msgs::msg::PointStamped& closest_point, size_t closest_idx)
{
    // 计算航向误差（即期望方向与当前方向的差值）
    double desired_heading = 0.0;
    
    // 通过下一个路径点计算期望方向
    if (closest_idx < center_line_.poses.size() - 1) {
        const auto& next_point = center_line_.poses[closest_idx + 1].pose.position;
        const auto& current_point = closest_point.point;
        
        double dx = next_point.x - current_point.x;
        double dy = next_point.y - current_point.y;
        desired_heading = atan2(dy, dx);
    } else if (closest_idx > 0) {
        // 如果是最后一个点，则使用前一个点计算方向
        const auto& prev_point = center_line_.poses[closest_idx - 1].pose.position;
        const auto& current_point = closest_point.point;
        
        double dx = current_point.x - prev_point.x;
        double dy = current_point.y - prev_point.y;
        desired_heading = atan2(dy, dx);
    }
    
    // 计算航向误差（规范化到[-π, π]）
    double heading_error = desired_heading - current_yaw_;
    while (heading_error > M_PI) heading_error -= 2 * M_PI;
    while (heading_error < -M_PI) heading_error += 2 * M_PI;
    
    return heading_error;
}

void PIDController::publish_lookahead_marker(const geometry_msgs::msg::PointStamped& point)
{
    visualization_msgs::msg::Marker marker;
    marker.header = point.header;
    marker.ns = "lookahead";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position = point.point;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);
    lookahead_marker_pub_->publish(marker);
}

geometry_msgs::msg::Twist PIDController::calculate_control_command()
{
    geometry_msgs::msg::Twist cmd_vel;
    
    // 调试模式：输出固定速度测试转向
    if (debug_mode_) {
        cmd_vel.linear.x = 0.1;
        cmd_vel.angular.z = 0.0;
        RCLCPP_INFO(this->get_logger(), "调试模式：线速度=0.1m/s, 角速度=0");
        return cmd_vel;
    }

    // 获取当前时间
    auto current_time = this->now();
    double dt = (current_time - last_time_).seconds();
    last_time_ = current_time;
    
    if (dt <= 0) dt = 0.01; // 防止除零错误

    // 1. 找到最近的路径点
    auto closest_point = find_closest_point();
    publish_lookahead_marker(closest_point);  // 可视化最近点

    // 2. 计算误差
    double cross_track_error = calculate_cross_track_error(closest_point);
    
    // 找到最近点的索引
    size_t closest_idx = 0;
    double min_dist = 1e9;
    for (size_t i = 0; i < center_line_.poses.size(); ++i) {
        const auto& p = center_line_.poses[i].pose.position;
        double dx = p.x - current_x_;
        double dy = p.y - current_y_;
        double dist = std::hypot(dx, dy);
        if (dist < min_dist) {
            min_dist = dist;
            closest_idx = i;
        }
    }
    
    double heading_error = calculate_heading_error(closest_point, closest_idx);
    
    // 总的角速度误差是横向误差和航向误差的组合
    double angular_error = cross_track_error + heading_error;

    // 3. 更新积分项
    linear_error_sum_ += cross_track_error * dt;
    angular_error_sum_ += angular_error * dt;

    // 4. 计算微分项
    double linear_error_diff = (cross_track_error - last_cross_track_error_) / dt;
    double angular_error_diff = (angular_error - last_heading_error_) / dt;
    
    last_cross_track_error_ = cross_track_error;
    last_heading_error_ = angular_error;

    // 5. PID控制计算
    // 线速度控制（主要基于纵向位置，这里简化为常数）
    double linear_speed = max_linear_speed_;
    
    // 角速度控制（PID控制）
    double angular_vel = kp_angular_ * angular_error + 
                         ki_angular_ * angular_error_sum_ + 
                         kd_angular_ * angular_error_diff;
    
    // 限制角速度范围
    angular_vel = std::clamp(angular_vel, -max_angular_speed_, max_angular_speed_);
    
    // 6. 弯道减速
    if (std::abs(angular_vel) > 0.2) {
        linear_speed *= (1.0 - 0.3 * std::abs(angular_vel) / max_angular_speed_);
    }
    linear_speed = std::max(linear_speed, min_linear_speed_);

    // 7. 赋值控制指令
    cmd_vel.linear.x = linear_speed;
    cmd_vel.angular.z = angular_vel;

    // 调试日志
    RCLCPP_DEBUG(this->get_logger(), 
        "横向误差: %.2f, 航向误差: %.2f, 线速度: %.2f, 角速度: %.2f",
        cross_track_error, heading_error, linear_speed, angular_vel);

    return cmd_vel;
}