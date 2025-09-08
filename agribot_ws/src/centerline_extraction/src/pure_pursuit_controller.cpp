#include "centerline_extraction/pure_pursuit_controller.hpp"
#include <cmath>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

PurePursuitController::PurePursuitController() : Node("pure_pursuit_controller"), has_center_line_(false)
{
    // 订阅玉米行中心线
    center_line_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/corn_row_center", 10,
        std::bind(&PurePursuitController::center_line_callback, this, std::placeholders::_1));
    
    // 订阅差速控制器里程计（注意控制器名称）
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,  // 匹配配置中的控制器名称
        std::bind(&PurePursuitController::odom_callback, this, std::placeholders::_1));
    
    // 发布速度指令（匹配配置中的控制器名称和指令类型）
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10);  // 非stamped指令
    
    // 初始化参数（与配置文件中的wheel_separation匹配）
    this->declare_parameter("lookahead_distance", 0.206);    // 适配小车尺寸（轮距0.18m）
    this->declare_parameter("max_linear_speed", 0.3);      // 低速启动
    this->declare_parameter("min_linear_speed", 0.05);
    this->declare_parameter("max_angular_speed", 2.0);     // 提高转向灵敏度
    this->declare_parameter("wheel_base", 0.206);           // 与配置中的wheel_separation一致
    this->declare_parameter("kp_lateral", 0.5);            // 增强转向纠正
    this->declare_parameter("debug_mode", false);          // 调试模式
    
    get_parameters();
    
    // 初始化状态
    current_x_ = current_y_ = current_yaw_ = 0.0;
    current_linear_vel_ = 0.0;
}

void PurePursuitController::get_parameters()
{
    this->get_parameter("lookahead_distance", lookahead_distance_);
    this->get_parameter("max_linear_speed", max_linear_speed_);
    this->get_parameter("min_linear_speed", min_linear_speed_);
    this->get_parameter("max_angular_speed", max_angular_speed_);
    this->get_parameter("wheel_base", wheel_base_);
    this->get_parameter("kp_lateral", kp_lateral_);
    this->get_parameter("debug_mode", debug_mode_);
}

void PurePursuitController::center_line_callback(const nav_msgs::msg::Path::SharedPtr msg)
{
    center_line_ = *msg;
    has_center_line_ = true;
    RCLCPP_DEBUG(this->get_logger(), "收到中心线，含%d个点", center_line_.poses.size());
}

void PurePursuitController::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    getCurrentPose(msg, current_x_, current_y_, current_yaw_);
    current_linear_vel_ = msg->twist.twist.linear.x;
    
    if (has_center_line_ && !center_line_.poses.empty())
    {
        auto cmd_vel = calculateControlCommand();
        cmd_vel_pub_->publish(cmd_vel);
    }
}

void PurePursuitController::getCurrentPose(const nav_msgs::msg::Odometry::SharedPtr odom,
                                          double& x, double& y, double& yaw)
{
    x = odom->pose.pose.position.x;
    y = odom->pose.pose.position.y;
    
    // 解析偏航角
    tf2::Quaternion q(
        odom->pose.pose.orientation.x,
        odom->pose.pose.orientation.y,
        odom->pose.pose.orientation.z,
        odom->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, yaw);
}

geometry_msgs::msg::PointStamped PurePursuitController::findLookaheadPoint()
{
    geometry_msgs::msg::PointStamped lookahead_point;
    lookahead_point.header.frame_id = center_line_.header.frame_id;
    lookahead_point.header.stamp = this->now();
    
    if (center_line_.poses.empty()) return lookahead_point;
    
    // 预瞄距离（适配小车尺寸）
    double dynamic_lookahead = lookahead_distance_ + 0.3 * current_linear_vel_;
    dynamic_lookahead = std::clamp(dynamic_lookahead, 0.4, 1.0);
    
    // 找最近路径点
    size_t closest_idx = 0;
    double min_dist = 1e9;
    for (size_t i = 0; i < center_line_.poses.size(); ++i)
    {
        const auto& p = center_line_.poses[i].pose.position;
        double dx = p.x - current_x_;
        double dy = p.y - current_y_;
        double dist = std::hypot(dx, dy);
        if (dist < min_dist)
        {
            min_dist = dist;
            closest_idx = i;
        }
    }
    closest_point_.point = center_line_.poses[closest_idx].pose.position;
    
    // 找预瞄点
    for (size_t i = closest_idx; i < center_line_.poses.size(); ++i)
    {
        const auto& p = center_line_.poses[i].pose.position;
        double dx = p.x - current_x_;
        double dy = p.y - current_y_;
        double dist = std::hypot(dx, dy);
        
        if (dist >= dynamic_lookahead)
        {
            if (i == 0)
            {
                lookahead_point.point = p;
                return lookahead_point;
            }
            const auto& p_prev = center_line_.poses[i-1].pose.position;
            double dx_prev = p_prev.x - current_x_;
            double dy_prev = p_prev.y - current_y_;
            double dist_prev = std::hypot(dx_prev, dy_prev);
            
            double t = (dynamic_lookahead - dist_prev) / (dist - dist_prev);
            lookahead_point.point.x = p_prev.x + t * (p.x - p_prev.x);
            lookahead_point.point.y = p_prev.y + t * (p.y - p_prev.y);
            return lookahead_point;
        }
    }
    
    lookahead_point.point = center_line_.poses.back().pose.position;
    return lookahead_point;
}

double PurePursuitController::calculateLateralError()
{
    double dx = closest_point_.point.x - current_x_;
    double dy = closest_point_.point.y - current_y_;
    return -sin(current_yaw_) * dx + cos(current_yaw_) * dy;  // Y左为正
}

geometry_msgs::msg::Twist PurePursuitController::calculateControlCommand()
{
    geometry_msgs::msg::Twist cmd_vel;
    
    // 调试模式：输出固定速度测试
    if (debug_mode_) {
        cmd_vel.linear.x = 0.1;  // 缓慢前进
        cmd_vel.angular.z = 0.5; // 缓慢转向
        RCLCPP_INFO(this->get_logger(), "调试模式：线速度=0.1, 角速度=0.5");
        return cmd_vel;
    }
    
    // 正常控制逻辑
    lookahead_point_ = findLookaheadPoint();
    
    double dx = lookahead_point_.point.x - current_x_;
    double dy = lookahead_point_.point.y - current_y_;
    double x_veh = cos(current_yaw_) * dx + sin(current_yaw_) * dy;
    double y_veh = -sin(current_yaw_) * dx + cos(current_yaw_) * dy;
    
    // 计算转向角
    double steering_angle = (2 * y_veh) / (x_veh*x_veh + y_veh*y_veh);
    steering_angle += kp_lateral_ * calculateLateralError();
    steering_angle = std::clamp(steering_angle, -max_angular_speed_, max_angular_speed_);
    
    // 计算线速度（弯道减速）
    double linear_speed = max_linear_speed_ * (1.0 - 0.8 * std::fabs(steering_angle)/max_angular_speed_);
    linear_speed = std::max(linear_speed, min_linear_speed_);
    
    // 赋值指令
    cmd_vel.linear.x = linear_speed;
    cmd_vel.angular.z = steering_angle;
    
    RCLCPP_DEBUG(this->get_logger(), "线速度: %.2f, 角速度: %.2f, 横向偏差: %.2f",
                 linear_speed, steering_angle, calculateLateralError());
    return cmd_vel;
}
    