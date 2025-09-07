#include "centerline_extraction/pure_pursuit_controller.hpp"
#include <cmath>

PurePursuitController::PurePursuitController() : Node("pure_pursuit_controller")
{
    // 声明参数
    declare_parameter("lookahead_distance", 1.0);
    declare_parameter("wheelbase", 0.5);  // 小车轴距（示例值）
    declare_parameter("max_linear_speed", 0.5);
    declare_parameter("max_angular_speed", 1.0);
    declare_parameter("control_frequency", 10.0);
    
    // 获取参数
    get_parameter("lookahead_distance", lookahead_distance_);
    get_parameter("wheelbase", wheelbase_);
    get_parameter("max_linear_speed", max_linear_speed_);
    get_parameter("max_angular_speed", max_angular_speed_);
    get_parameter("control_frequency", control_frequency_);
    
    // 创建订阅者和发布者
    center_line_sub_ = create_subscription<nav_msgs::msg::Path>(
        "/corn_row_center", 10,
        std::bind(&PurePursuitController::centerLineCallback, this, std::placeholders::_1)
    );
    
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&PurePursuitController::odometryCallback, this, std::placeholders::_1)
    );
    
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    
    // 创建控制循环定时器
    control_timer_ = create_wall_timer(
        std::chrono::milliseconds((int)(1000.0 / control_frequency_)),
        std::bind(&PurePursuitController::controlLoop, this)
    );
    
    path_received_ = false;
    odom_received_ = false;
    
    RCLCPP_INFO(get_logger(), "Pure pursuit controller initialized");
}

void PurePursuitController::centerLineCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
    current_path_ = *msg;
    path_received_ = true;
}

void PurePursuitController::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    current_odom_ = *msg;
    odom_received_ = true;
}

geometry_msgs::msg::PoseStamped PurePursuitController::findLookaheadPoint()
{
    geometry_msgs::msg::PoseStamped lookahead_point;
    lookahead_point.header = current_path_.header;
    
    if (current_path_.poses.empty())
    {
        return lookahead_point;
    }
    
    float current_x, current_y, current_yaw;
    getCurrentPose(std::make_shared<nav_msgs::msg::Odometry>(current_odom_), 
                  current_x, current_y, current_yaw);
    
    // 寻找第一个距离大于预瞄距离的点
    for (const auto& pose : current_path_.poses)
    {
        float dist = distanceBetweenPoints(
            current_x, current_y,
            pose.pose.position.x, pose.pose.position.y
        );
        
        if (dist >= lookahead_distance_)
        {
            return pose;
        }
    }
    
    // 如果所有点都太近，返回最后一个点
    return current_path_.poses.back();
}

void PurePursuitController::getCurrentPose(const nav_msgs::msg::Odometry::SharedPtr msg, 
                                          float& x, float& y, float& yaw)
{
    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;
    
    // 从四元数计算偏航角
    tf2::Quaternion quat(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w
    );
    tf2::Matrix3x3 m(quat);
    double roll, pitch, yaw_double;
    m.getRPY(roll, pitch, yaw_double);
    yaw = static_cast<float>(yaw_double);
}

float PurePursuitController::distanceBetweenPoints(float x1, float y1, float x2, float y2)
{
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}

void PurePursuitController::controlLoop()
{
    // 等待路径和里程计数据
    if (!path_received_ || !odom_received_)
    {
        return;
    }
    
    // 如果路径为空，停止小车
    if (current_path_.poses.empty())
    {
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        cmd_vel_pub_->publish(cmd_vel);
        return;
    }
    
    // 获取当前姿态
    float current_x, current_y, current_yaw;
    getCurrentPose(std::make_shared<nav_msgs::msg::Odometry>(current_odom_), 
                  current_x, current_y, current_yaw);
    
    // 找到预瞄点
    auto lookahead_point = findLookaheadPoint();
    
    // 将预瞄点转换到小车坐标系
    float dx = lookahead_point.pose.position.x - current_x;
    float dy = lookahead_point.pose.position.y - current_y;
    
    // 旋转到小车局部坐标系
    float dx_local = dx * cos(current_yaw) + dy * sin(current_yaw);
    float dy_local = -dx * sin(current_yaw) + dy * cos(current_yaw);
    
    // 计算转向角 (纯追踪算法)
    float L = sqrt(dx_local*dx_local + dy_local*dy_local);  // 实际预瞄距离
    float steering_angle = 2 * dy_local / (L * L);
    
    // 计算角速度 (简化处理)
    float angular_vel = steering_angle * (max_linear_speed_ / wheelbase_);
    
    // 限制角速度
    angular_vel = std::clamp(angular_vel, -max_angular_speed_, max_angular_speed_);
    
    // 创建控制命令
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = max_linear_speed_;  // 固定线速度，可根据环境动态调整
    cmd_vel.angular.z = angular_vel;
    
    // 发布控制命令
    cmd_vel_pub_->publish(cmd_vel);
    
    RCLCPP_DEBUG(get_logger(), "Linear: %f, Angular: %f", 
                cmd_vel.linear.x, cmd_vel.angular.z);
}
