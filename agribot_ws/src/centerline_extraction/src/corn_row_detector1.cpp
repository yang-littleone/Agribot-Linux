#include "centerline_extraction/corn_row_detector.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/centroid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cmath>
#include <algorithm>
#include <numeric>

// 构造函数
CornRowDetector::CornRowDetector() : Node("corn_row_detector")
{
    // 初始化订阅者和发布者
    point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/mid360_PointCloud2", 10,
        std::bind(&CornRowDetector::point_cloud_callback, this, std::placeholders::_1));
    
    center_line_pub_ = this->create_publisher<nav_msgs::msg::Path>("/corn_row_center", 10);
    
    // 声明参数
    this->declare_parameter("cluster_tolerance", 0.2f);
    this->declare_parameter("min_cluster_size", 10);
    this->declare_parameter("max_cluster_size", 200);
    this->declare_parameter("row_distance_threshold", 0.5f);
    this->declare_parameter("y_crop_min", -3.0f);
    this->declare_parameter("y_crop_max", 3.0f);
    this->declare_parameter("z_min", 0.1f);
    this->declare_parameter("z_max", 1.0f);
    this->declare_parameter("fit_distance", 2.0f);  // 前方拟合距离（米）
    this->declare_parameter("temporal_window_size", 3);  // 时间平滑窗口
    
    // 获取参数
    this->get_parameter("cluster_tolerance", cluster_tolerance_);
    this->get_parameter("min_cluster_size", min_cluster_size_);
    this->get_parameter("max_cluster_size", max_cluster_size_);
    this->get_parameter("row_distance_threshold", row_distance_threshold_);
    this->get_parameter("y_crop_min", y_crop_min_);
    this->get_parameter("y_crop_max", y_crop_max_);
    this->get_parameter("z_min", z_min_);
    this->get_parameter("z_max", z_max_);
    this->get_parameter("fit_distance", fit_distance_);
    this->get_parameter("temporal_window_size", temporal_window_size_);

    RCLCPP_INFO(this->get_logger(), "直线拟合玉米行检测器初始化完成（前方%.1f米范围）", fit_distance_);
}

// 点云回调函数
void CornRowDetector::point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    // 转换ROS点云到PCL格式
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *cloud);

    // 预处理
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud = preprocess_cloud(cloud);
    if (filtered_cloud->empty()) {
        RCLCPP_WARN(this->get_logger(), "预处理后无有效点云");
        return;
    }

    // 玉米株聚类
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters = cluster_plants(filtered_cloud);
    if (clusters.empty()) {
        RCLCPP_WARN(this->get_logger(), "未检测到玉米株");
        return;
    }

    // 分组为玉米行
    std::vector<std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>> rows = group_into_rows(clusters);
    if (rows.size() < 2) {
        RCLCPP_WARN(this->get_logger(), "检测到不足2行玉米（%zu行）", rows.size());
        return;
    }

    // 计算中心线（限制前方2米）
    nav_msgs::msg::Path raw_path = compute_center_line(rows);
    raw_path.header = msg->header;

    // 平滑处理
    nav_msgs::msg::Path smoothed_path = smooth_path(raw_path);

    // 发布
    center_line_pub_->publish(smoothed_path);
}

// 点云预处理
pcl::PointCloud<pcl::PointXYZ>::Ptr CornRowDetector::preprocess_cloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr input)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());

    // 高度过滤（玉米株高度）
    pcl::PassThrough<pcl::PointXYZ> pass_z;
    pass_z.setInputCloud(input);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(z_min_, z_max_);
    pass_z.filter(*cloud_filtered);

    // 横向范围过滤
    pcl::PassThrough<pcl::PointXYZ> pass_y;
    pass_y.setInputCloud(cloud_filtered);
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(y_crop_min_, y_crop_max_);
    pass_y.filter(*cloud_filtered);

    // 降采样
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setInputCloud(cloud_filtered);
    voxel_grid.setLeafSize(0.05f, 0.05f, 0.05f);
    voxel_grid.filter(*cloud_filtered);

    return cloud_filtered;
}

// 玉米株聚类
std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> CornRowDetector::cluster_plants(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(cluster_tolerance_);  // 株内点距离
    ec.setMinClusterSize(min_cluster_size_);
    ec.setMaxClusterSize(max_cluster_size_);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    // 提取聚类结果
    for (const auto& indices : cluster_indices) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>());
        for (int idx : indices.indices) {
            cluster->points.push_back(cloud->points[idx]);
        }
        cluster->width = cluster->size();
        cluster->height = 1;
        cluster->is_dense = true;
        clusters.push_back(cluster);
    }

    return clusters;
}

// 玉米行分组
std::vector<std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>> CornRowDetector::group_into_rows(
    const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clusters)
{
    std::vector<std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>> rows;
    if (clusters.empty()) return rows;

    // 计算每个玉米株的中心点
    std::vector<pcl::PointXYZ> centroids;
    for (const auto& cluster : clusters) {
        pcl::PointXYZ c;
        pcl::computeCentroid(*cluster, c);
        centroids.push_back(c);
    }

    // 按y坐标聚类分组（同一行y相近）
    std::vector<bool> assigned(clusters.size(), false);
    for (size_t i = 0; i < clusters.size(); ++i) {
        if (assigned[i]) continue;

        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> row;
        row.push_back(clusters[i]);
        assigned[i] = true;

        // 寻找同一行的其他玉米株
        for (size_t j = i + 1; j < clusters.size(); ++j) {
            if (assigned[j]) continue;
            float dy = std::fabs(centroids[i].y - centroids[j].y);
            if (dy < row_distance_threshold_) {
                row.push_back(clusters[j]);
                assigned[j] = true;
            }
        }
        rows.push_back(row);
    }

    // 按行的y坐标排序（左到右）
    std::sort(rows.begin(), rows.end(), [this](const auto& a, const auto& b) {
        return get_row_centroid(a).y < get_row_centroid(b).y;
    });

    return rows;
}

// 行中心点计算
pcl::PointXYZ CornRowDetector::get_row_centroid(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& row)
{
    pcl::PointXYZ centroid(0, 0, 0);
    int count = 0;
    for (const auto& cluster : row) {
        for (const auto& p : cluster->points) {
            centroid.x += p.x;
            centroid.y += p.y;
            count++;
        }
    }
    if (count > 0) {
        centroid.x /= count;
        centroid.y /= count;
    }
    return centroid;
}

// 提取行内所有点
std::vector<pcl::PointXYZ> CornRowDetector::get_all_points(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& row)
{
    std::vector<pcl::PointXYZ> points;
    for (const auto& cluster : row) {
        points.insert(points.end(), cluster->points.begin(), cluster->points.end());
    }
    return points;
}

// 直线拟合（y = kx + b）
CornRowDetector::LineParam CornRowDetector::fit_linear(const std::vector<pcl::PointXYZ>& points)
{
    LineParam param{0.0f, 0.0f};
    if (points.size() < 2) return param;

    // 最小二乘法计算直线参数
    float sum_x = 0, sum_y = 0, sum_xx = 0, sum_xy = 0;
    int n = points.size();
    for (const auto& p : points) {
        sum_x += p.x;
        sum_y += p.y;
        sum_xx += p.x * p.x;
        sum_xy += p.x * p.y;
    }

    // 计算斜率k和截距b
    float denominator = n * sum_xx - sum_x * sum_x;
    if (std::fabs(denominator) < 1e-6) return param;  // 避免除零

    param.k = (n * sum_xy - sum_x * sum_y) / denominator;
    param.b = (sum_y - param.k * sum_x) / n;

    return param;
}

// 计算直线上的y值
float CornRowDetector::evaluate_line(const LineParam& line, float x)
{
    return line.k * x + line.b;
}

// 计算中心线（限制前方2米）
nav_msgs::msg::Path CornRowDetector::compute_center_line(const std::vector<std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>>& rows)
{
    nav_msgs::msg::Path path;
    if (rows.size() < 2) return path;

    // 获取左右两行点并排序
    auto left_points = get_all_points(rows[0]);
    auto right_points = get_all_points(rows[1]);
    if (left_points.empty() || right_points.empty()) return path;

    std::sort(left_points.begin(), left_points.end(), [](const auto& a, const auto& b) {
        return a.x < b.x;  // 按行进方向排序
    });
    std::sort(right_points.begin(), right_points.end(), [](const auto& a, const auto& b) {
        return a.x < b.x;
    });

    // 确定拟合范围：当前点（最近点）至前方fit_distance米
    float current_x = std::min(left_points.front().x, right_points.front().x);
    float max_x = current_x + fit_distance_;  // 前方2米

    // 截取范围内的点
    auto crop_to_range = [current_x, max_x](std::vector<pcl::PointXYZ>& points) {
        std::vector<pcl::PointXYZ> cropped;
        for (const auto& p : points) {
            if (p.x >= current_x && p.x <= max_x) {
                cropped.push_back(p);
            }
        }
        return cropped;
    };

    auto left_cropped = crop_to_range(left_points);
    auto right_cropped = crop_to_range(right_points);
    if (left_cropped.empty() || right_cropped.empty()) return path;

    // 左右行分别直线拟合
    LineParam left_line = fit_linear(left_cropped);
    LineParam right_line = fit_linear(right_cropped);

    // 生成中心线点（每隔0.2米一个点）
    int num_points = static_cast<int>((max_x - current_x) / 0.2f);
    for (int i = 0; i <= num_points; ++i) {
        float x = current_x + (max_x - current_x) * i / num_points;
        
        // 计算左右行在x处的y值
        float y_left = evaluate_line(left_line, x);
        float y_right = evaluate_line(right_line, x);

        // 计算中心线点
        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x = x;
        pose.pose.position.y = (y_left + y_right) / 2.0f;  // 左右中点
        pose.pose.position.z = 0.0f;

        // 计算方向角（沿直线方向）
        float dx = 0.5f;  // 微小增量
        float y_left_next = evaluate_line(left_line, x + dx);
        float y_right_next = evaluate_line(right_line, x + dx);
        float dy = ((y_left_next + y_right_next) / 2.0f) - pose.pose.position.y;
        float yaw = std::atan2(dy, dx);  // 方向角

        // 设置朝向
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        pose.pose.orientation = tf2::toMsg(q);

        path.poses.push_back(pose);
    }

    return path;
}

// 路径平滑处理
nav_msgs::msg::Path CornRowDetector::smooth_path(const nav_msgs::msg::Path& raw_path)
{
    nav_msgs::msg::Path smooth_path = raw_path;
    if (raw_path.poses.empty()) return smooth_path;

    // 1. 异常值剔除
    auto filtered = remove_outliers(raw_path.poses);
    if (filtered.empty()) return smooth_path;

    // 2. 时间平滑
    auto temporal_smoothed = temporal_smoothing(filtered);
    smooth_path.poses = temporal_smoothed;

    return smooth_path;
}

// 异常值剔除
std::vector<geometry_msgs::msg::PoseStamped> CornRowDetector::remove_outliers(
    const std::vector<geometry_msgs::msg::PoseStamped>& points)
{
    if (points.size() < 3) return points;

    // 计算y方向均值和标准差
    std::vector<float> ys;
    for (const auto& p : points) ys.push_back(p.pose.position.y);
    float mean = std::accumulate(ys.begin(), ys.end(), 0.0f) / ys.size();
    
    float sq_sum = std::inner_product(ys.begin(), ys.end(), ys.begin(), 0.0f,
        [](float a, float b) { return a + b; },
        [mean](float y, float) { return (y - mean) * (y - mean); });
    float std_dev = std::sqrt(sq_sum / ys.size());

    // 保留在均值±1.5倍标准差范围内的点
    std::vector<geometry_msgs::msg::PoseStamped> filtered;
    for (const auto& p : points) {
        if (std::fabs(p.pose.position.y - mean) <= 1.5f * std_dev) {
            filtered.push_back(p);
        }
    }

    return filtered.empty() ? points : filtered;
}

// 时间平滑
std::vector<geometry_msgs::msg::PoseStamped> CornRowDetector::temporal_smoothing(
    const std::vector<geometry_msgs::msg::PoseStamped>& current_poses)
{
    if (current_poses.empty()) return current_poses;

    // 维护滑动窗口
    while (static_cast<int>(path_buffer_.size()) > temporal_window_size_) {
        path_buffer_.pop_front();
    }
    path_buffer_.push_back(current_poses);

    // 窗口未满时直接返回当前帧
    if (static_cast<int>(path_buffer_.size()) < temporal_window_size_) {
        return current_poses;
    }

    // 多帧平均平滑
    std::vector<geometry_msgs::msg::PoseStamped> smoothed = current_poses;
    size_t min_size = current_poses.size();
    for (const auto& p : path_buffer_) {
        min_size = std::min(min_size, p.size());
    }

    // 对每个点进行时间域平均
    for (size_t i = 0; i < min_size; ++i) {
        float sum_x = 0.0f, sum_y = 0.0f;
        int count = 0;

        for (const auto& buf : path_buffer_) {
            if (i < buf.size()) {
                sum_x += buf[i].pose.position.x;
                sum_y += buf[i].pose.position.y;
                count++;
            }
        }

        if (count > 0) {
            smoothed[i].pose.position.x = sum_x / count;
            smoothed[i].pose.position.y = sum_y / count;
        }
    }

    return smoothed;
}
