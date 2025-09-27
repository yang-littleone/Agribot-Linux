#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"

class PIDController : public rclcpp::Node
{
public:
    PIDController();

private:
    // 回调函数
    void center_line_callback(const nav_msgs::msg::Path::SharedPtr msg);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

    // 核心控制函数
    geometry_msgs::msg::Twist calculate_control_command();
    geometry_msgs::msg::PointStamped find_closest_point();
    double calculate_cross_track_error(const geometry_msgs::msg::PointStamped& closest_point);
    double calculate_heading_error(const geometry_msgs::msg::PointStamped& closest_point, size_t closest_idx);

    // 辅助函数
    void get_parameters();
    void publish_lookahead_marker(const geometry_msgs::msg::PointStamped& point);

    // 订阅者和发布者
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr center_line_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr lookahead_marker_pub_;

    // PID控制器参数
    double kp_linear_;              // 线速度比例系数
    double ki_linear_;              // 线速度积分系数
    double kd_linear_;              // 线速度微分系数
    
    double kp_angular_;             // 角速度比例系数
    double ki_angular_;             // 角速度积分系数
    double kd_angular_;             // 角速度微分系数

    double max_linear_speed_;       // 最大线速度
    double min_linear_speed_;       // 最小线速度
    double max_angular_speed_;      // 最大角速度
    
    bool debug_mode_;               // 调试模式

    // 状态变量
    nav_msgs::msg::Path center_line_;  // 中心线路径
    bool has_center_line_;             // 是否收到中心线
    double current_x_;                 // 当前x坐标
    double current_y_;                 // 当前y坐标
    double current_yaw_;               // 当前偏航角
    double current_linear_vel_;        // 当前线速度
    
    // PID控制器状态变量
    double linear_error_sum_;          // 线速度误差积分
    double angular_error_sum_;         // 角速度误差积分
    double last_cross_track_error_;    // 上一次横向误差
    double last_heading_error_;        // 上一次航向误差
    
    geometry_msgs::msg::PointStamped closest_point_;  // 路径上最近的点
    geometry_msgs::msg::PointStamped lookahead_point_; // 前视目标点
    
    rclcpp::Time last_time_;           // 上一次控制时间
};

#endif  // PID_CONTROLLER_HPP