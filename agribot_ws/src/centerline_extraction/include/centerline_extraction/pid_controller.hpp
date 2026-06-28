#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "std_msgs/msg/float32.hpp"
#include "visualization_msgs/msg/marker.hpp"

class PIDController : public rclcpp::Node
{
public:
    PIDController();

private:
    // 回调函数
    void center_line_callback(const nav_msgs::msg::Path::SharedPtr msg);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void confidence_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void safety_margin_callback(const std_msgs::msg::Float32::SharedPtr msg);

    // 核心控制函数
    geometry_msgs::msg::PointStamped find_target_point();
    geometry_msgs::msg::Twist calculate_control_command();

    // PID辅助函数
    double compute_pid(double error, double dt, double &integral, double &previous_error, 
                      double kp, double ki, double kd);

    // 辅助函数
    void get_parameters();
    void publish_target_marker(const geometry_msgs::msg::PointStamped& point);
    double compute_confidence_factor() const;
    double compute_safety_factor() const;
    bool should_stop_for_safety() const;

    // 订阅者和发布者
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr center_line_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr confidence_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr safety_margin_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr target_marker_pub_;

    // 控制器参数
    double target_distance_;        // 目标跟随距离（距离路径前方多少米）
    double max_linear_speed_;       // 最大线速度
    double min_linear_speed_;       // 最小线速度
    double max_angular_speed_;      // 最大角速度
    double confidence_high_threshold_;
    double confidence_low_threshold_;
    double confidence_stop_threshold_;
    double confidence_min_speed_factor_;
    double safety_margin_high_;
    double safety_margin_mid_;
    double safety_margin_stop_;
    int max_low_confidence_frames_;
    bool use_quality_aware_control_;
    
    // PID参数
    double lateral_kp_;             // 横向控制比例系数
    double lateral_ki_;             // 横向控制积分系数
    double lateral_kd_;             // 横向控制微分系数
    
    double heading_kp_;             // 航向控制比例系数
    double heading_ki_;             // 航向控制积分系数
    double heading_kd_;             // 航向控制微分系数

    bool debug_mode_;               // 调试模式

    // 状态变量
    nav_msgs::msg::Path center_line_;  // 中心线路径
    nav_msgs::msg::Path last_valid_center_line_;
    bool has_center_line_;             // 是否收到中心线
    bool has_last_valid_center_line_;
    bool use_recovery_path_;
    int low_confidence_count_;
    double corridor_confidence_;
    double corridor_safety_margin_;
    bool has_corridor_confidence_;
    bool has_corridor_safety_margin_;
    double current_x_;                 // 当前x坐标
    double current_y_;                 // 当前y坐标
    double current_yaw_;               // 当前偏航角
    double current_linear_vel_;        // 当前线速度
    double current_angular_vel_;       // 当前角速度
    
    geometry_msgs::msg::PointStamped target_point_; // 目标点
    
    // PID控制器状态变量
    double lateral_integral_;
    double lateral_previous_error_;
    double heading_integral_;
    double heading_previous_error_;
    
    // 时间记录
    rclcpp::Time last_time_;
    bool first_run_;

};

#endif  // PID_CONTROLLER_HPP
