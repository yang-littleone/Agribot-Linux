#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_types.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/filters/passthrough.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/segmentation/extract_clusters.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/common/centroid.h"

#include "centerline_extraction/corn_row_detector.hpp"

/* 直线拟合初代版本 */
CornRowDetector::CornRowDetector() : Node("corn_row_detector")
{
    // 订阅点云数据
    point_cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "/mid360_PointCloud2", 10,
        std::bind(&CornRowDetector::point_cloud_callback, this, std::placeholders::_1));

    // 发布中心线
    center_line_pub_ = create_publisher<nav_msgs::msg::Path>("/corn_row_center", 10);

    // 参数设置
    declare_parameter("cluster_tolerance", 0.3); // 聚类距离阈值
    declare_parameter("min_cluster_size", 5);    // 最小聚类点数
    declare_parameter("max_cluster_size", 100);  // 最大聚类点数
    declare_parameter("max_distance", 5.0);      // 检测最大距离
    declare_parameter("row_width_min", 0.5);     // 最小行距
    declare_parameter("row_width_max", 1.0);     // 最大行距

    CornRowDetector::get_parameters();
}

void CornRowDetector::get_parameters()
{
    get_parameter("cluster_tolerance", cluster_tolerance_);
    get_parameter("min_cluster_size", min_cluster_size_);
    get_parameter("max_cluster_size", max_cluster_size_);
    get_parameter("max_distance", max_distance_);
    get_parameter("row_width_min", row_width_min_);
    get_parameter("row_width_max", row_width_max_);
}

void CornRowDetector::point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    // 转换为PCL点云格式
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    // 1. 点云预处理 - 降采样
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setInputCloud(cloud);
    voxel_grid.setLeafSize(0.15f, 0.15f, 0.15f); // 5cm体素
    voxel_grid.filter(*cloud_filtered);

    // 2. 裁剪感兴趣区域 - 只保留前方一定距离
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cropped(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud_filtered);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(0.0, max_distance_); // 只保留x轴0到max_distance范围内的点
    pass.filter(*cloud_cropped);

    // 3. 地面分割 using RANSAC
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.1); // 地面点阈值

    seg.setInputCloud(cloud_cropped);
    seg.segment(*inliers, *coefficients);

    // 提取非地面点（玉米植株）
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plants(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud_cropped);
    extract.setIndices(inliers);
    extract.setNegative(true); // 提取非地面点
    extract.filter(*cloud_plants);

    // 4. 聚类检测玉米植株 - 使用EuclideanClusterExtraction替代DBSCAN
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_plants);

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(cluster_tolerance_); // 聚类距离阈值
    ec.setMinClusterSize(min_cluster_size_);    // 最小聚类点数
    ec.setMaxClusterSize(max_cluster_size_);    // 最大聚类点数
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_plants);
    ec.extract(cluster_indices);

    // 5. 计算每个聚类的中心点
    std::vector<Eigen::Vector3f> cluster_centers;
    for (const auto &indices : cluster_indices)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for (int index : indices.indices)
        {
            cluster->points.push_back(cloud_plants->points[index]);
        }

        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cluster, centroid);
        cluster_centers.emplace_back(centroid[0], centroid[1], centroid[2]);
    }

    // 6. 检测左右玉米行并计算中心线
    if (cluster_centers.size() >= 2)
    {
        auto [left_line, right_line] = detect_left_right_rows(cluster_centers);
        auto center_line = compute_center_line(left_line, right_line);

        // 发布中心线
        publish_center_line(center_line, msg->header);
    }
}

// 检测左右两行玉米
std::pair<std::vector<Eigen::Vector3f>, std::vector<Eigen::Vector3f>>
CornRowDetector::detect_left_right_rows(const std::vector<Eigen::Vector3f> &centers)
{
    std::vector<Eigen::Vector3f> left_points, right_points;

    // 简单实现：按y坐标聚类为左右两组
    // 更复杂的实现可以使用线性回归拟合
    std::vector<float> y_coords;
    for (const auto &c : centers)
        y_coords.push_back(c.y());

    // 找到中位数作为分割点
    std::sort(y_coords.begin(), y_coords.end());
    float median_y = y_coords[y_coords.size() / 2];

    // 分割为左右两组
    for (const auto &c : centers)
    {
        if (c.y() < median_y)
            left_points.push_back(c);
        else
            right_points.push_back(c);
    }

    // 如果分组不合理，尝试调整
    if (left_points.empty() || right_points.empty())
    {
        // 简单处理：按最左和最右分组
        auto min_it = std::min_element(centers.begin(), centers.end(),
                                       [](const Eigen::Vector3f &a, const Eigen::Vector3f &b)
                                       { return a.y() < b.y(); });
        auto max_it = std::max_element(centers.begin(), centers.end(),
                                       [](const Eigen::Vector3f &a, const Eigen::Vector3f &b)
                                       { return a.y() < b.y(); });

        left_points.push_back(*min_it);
        right_points.push_back(*max_it);
    }

    return {left_points, right_points};
}

// 计算两行之间的中心线
std::vector<Eigen::Vector3f> CornRowDetector::compute_center_line(
    const std::vector<Eigen::Vector3f> &left_points,
    const std::vector<Eigen::Vector3f> &right_points)
{
    std::vector<Eigen::Vector3f> center_line;

    // 对左右点按x坐标排序（沿行驶方向）
    std::vector<Eigen::Vector3f> sorted_left = left_points;
    std::vector<Eigen::Vector3f> sorted_right = right_points;

    std::sort(sorted_left.begin(), sorted_left.end(),
              [](const Eigen::Vector3f &a, const Eigen::Vector3f &b)
              { return a.x() < b.x(); });
    std::sort(sorted_right.begin(), sorted_right.end(),
              [](const Eigen::Vector3f &a, const Eigen::Vector3f &b)
              { return a.x() < b.x(); });

    // 计算中心线点
    size_t max_size = std::max(sorted_left.size(), sorted_right.size());
    for (size_t i = 0; i < max_size; ++i)
    {
        Eigen::Vector3f left = (i < sorted_left.size()) ? sorted_left[i] : sorted_left.back();
        Eigen::Vector3f right = (i < sorted_right.size()) ? sorted_right[i] : sorted_right.back();

        // 计算中点
        Eigen::Vector3f center;
        center.x() = (left.x() + right.x()) / 2.0f;
        center.y() = (left.y() + right.y()) / 2.0f;
        center.z() = 0.0f; // 地面高度

        center_line.push_back(center);
    }

    return center_line;
}

// 发布中心线
void CornRowDetector::publish_center_line(const std::vector<Eigen::Vector3f> &center_line,
                                          const std_msgs::msg::Header &header)
{
    nav_msgs::msg::Path path_msg;
    path_msg.header = header;

    for (const auto &point : center_line)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = header;
        pose.pose.position.x = point.x();
        pose.pose.position.y = point.y();
        pose.pose.position.z = point.z();
        pose.pose.orientation.w = 1.0; // 无旋转

        path_msg.poses.push_back(pose);
    }

    center_line_pub_->publish(path_msg);
}