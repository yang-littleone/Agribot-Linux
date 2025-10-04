#ifndef CORN_ROW_DETECTOR_PROJECTION_HPP
#define CORN_ROW_DETECTOR_PROJECTION_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include <deque>

// define point cloud type
using PointCloudXYZ = pcl::PointCloud<pcl::PointXYZ>;
using PointCloudXYZPtr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

class CornRowDetectorProjection : public rclcpp::Node
{
private:
    // define subscriber and publisher
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr left_row_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr right_row_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr center_line_pub_;
    
    float z_min_ = 0.0;
    float z_max_ = 0.5;
    float voxel_size_ = 0.02;
    
    // 平滑处理参数
    int time_window_size_ = 5;          // 时间窗口大小
    float spatial_smooth_weight_ = 0.7; // 空间平滑权重
    float outlier_threshold_ = 2.0;     // 异常值剔除阈值(标准差倍数)
    
    // 路径历史缓存
    std::deque<nav_msgs::msg::Path> path_history_;

public:
    CornRowDetectorProjection();

private:
    // the callback of point cloud subscriber
    void point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    // preprocess point cloud
    PointCloudXYZPtr preprocess_point_cloud(PointCloudXYZPtr input_cloud);

    // projection point cloud to xy plane
    PointCloudXYZPtr projection_point_cloud(PointCloudXYZPtr input_cloud);
    
    // split point cloud to left and right rows
    std::pair<PointCloudXYZPtr, PointCloudXYZPtr> split_left_right_rows(PointCloudXYZPtr input_cloud);
    
    // fit line for point cloud
    std::pair<float, float> fit_line(PointCloudXYZPtr cloud);
    
    // create path from line parameters
    nav_msgs::msg::Path create_path(float slope, float intercept, const std_msgs::msg::Header& header);
    
    // 平滑处理函数
    nav_msgs::msg::Path smooth_path(const nav_msgs::msg::Path& raw_path);
    
    // 异常值剔除
    nav_msgs::msg::Path remove_outliers(const nav_msgs::msg::Path& path);
    
    // 空间平滑
    nav_msgs::msg::Path spatial_smoothing(const nav_msgs::msg::Path& path);
    
    // 时间平滑
    nav_msgs::msg::Path temporal_smoothing(const nav_msgs::msg::Path& path);
};

#endif