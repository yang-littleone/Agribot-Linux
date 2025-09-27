#ifndef CORN_ROW_DETECTOR_PROJECTION_HPP
#define CORN_ROW_DETECTOR_PROJECTION_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pcl/point_cloud.h" // provide pcl point cloud type
#include "pcl/point_types.h" // provide pcl point types

// define point cloud type
using PointCloudXYZ = pcl::PointCloud<pcl::PointXYZ>;
using PointCloudXYZPtr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

class CornRowDetectorProjection : public rclcpp::Node
{
private:
    // define subscriber and publisher
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr center_line_pub_;
    
    float z_min_ = 0.0;
    float z_max_ = 0.5;
    float voxel_size_ = 0.02;

public:
    CornRowDetectorProjection();

private:
    // the callback of point cloud subscriber
    void point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    // preprocess point cloud
    PointCloudXYZPtr preprocess_point_cloud(PointCloudXYZPtr input_cloud);

    // projection point cloud to xy plane
    PointCloudXYZPtr projection_point_cloud(PointCloudXYZPtr input_cloud);
};

#endif