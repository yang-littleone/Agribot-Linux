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

// 构造函数实现
// 构造函数实现
CornRowDetector::CornRowDetector() : Node("corn_row_detector")
{
    // 初始化订阅者和发布者
    point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/mid360_PointCloud2", 10,
        std::bind(&CornRowDetector::point_cloud_callback, this, std::placeholders::_1));
    
    center_line_pub_ = this->create_publisher<nav_msgs::msg::Path>("/corn_row_center", 10);
    
    // 声明并获取参数 - 修复类型歧义问题
    this->declare_parameter("cluster_tolerance", 0.3f);
    this->declare_parameter("min_cluster_size", 10);
    this->declare_parameter("max_cluster_size", 200);
    this->declare_parameter("row_distance_threshold", 0.5f);
    this->declare_parameter("y_crop_min", -2.0f);
    this->declare_parameter("y_crop_max", 2.0f);
    this->declare_parameter("z_min", 0.1f);
    this->declare_parameter("z_max", 1.0f);
    
    // 平滑参数（使用int避免无符号类型歧义）
    this->declare_parameter("time_window_size", 8);  // 改为int类型避免歧义
    this->declare_parameter("spatial_smooth_weight", 0.7f);
    this->declare_parameter("max_jump_distance", 0.1f);
    this->declare_parameter("outlier_std_threshold", 1.0f);
    this->declare_parameter("spline_segments", 8);
    
    // 获取参数
    this->get_parameter("cluster_tolerance", cluster_tolerance_);
    this->get_parameter("min_cluster_size", min_cluster_size_);
    this->get_parameter("max_cluster_size", max_cluster_size_);
    this->get_parameter("row_distance_threshold", row_distance_threshold_);
    this->get_parameter("y_crop_min", y_crop_min_);
    this->get_parameter("y_crop_max", y_crop_max_);
    this->get_parameter("z_min", z_min_);
    this->get_parameter("z_max", z_max_);
    
    this->get_parameter("time_window_size", time_window_size_);
    this->get_parameter("spatial_smooth_weight", spatial_smooth_weight_);
    this->get_parameter("max_jump_distance", max_jump_distance_);
    this->get_parameter("outlier_std_threshold", outlier_std_threshold_);
    this->get_parameter("spline_segments", spline_segments_);

    RCLCPP_INFO(this->get_logger(), "玉米行检测器初始化完成（平滑版）");
}


// 点云回调函数实现
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

    // 聚类
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters = cluster_plants(filtered_cloud);
    if (clusters.empty()) {
        RCLCPP_WARN(this->get_logger(), "未检测到玉米株");
        return;
    }

    // 分组为行
    std::vector<std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>> rows = group_into_rows(clusters);
    if (rows.size() < 2) {
        RCLCPP_WARN(this->get_logger(), "检测到不足2行玉米（%zu行）", rows.size());
        return;
    }

    // 计算原始中心线
    nav_msgs::msg::Path raw_path = compute_center_line(rows);
    raw_path.header = msg->header;

    // 高级平滑处理
    nav_msgs::msg::Path smooth_path = advanced_smoothing(raw_path);

    // 发布
    center_line_pub_->publish(smooth_path);
}

// 点云预处理实现
pcl::PointCloud<pcl::PointXYZ>::Ptr CornRowDetector::preprocess_cloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr input)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());

    // 高度过滤
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

// 玉米株聚类实现
std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> CornRowDetector::cluster_plants(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(cluster_tolerance_);
    ec.setMinClusterSize(min_cluster_size_);
    ec.setMaxClusterSize(max_cluster_size_);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

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

// 分组为玉米行实现
std::vector<std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>> CornRowDetector::group_into_rows(
    const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clusters)
{
    std::vector<std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>> rows;
    if (clusters.empty()) return rows;

    std::vector<pcl::PointXYZ> centroids;
    for (const auto& cluster : clusters) {
        pcl::PointXYZ c;
        pcl::computeCentroid(*cluster, c);
        centroids.push_back(c);
    }

    std::vector<bool> assigned(clusters.size(), false);
    for (size_t i = 0; i < clusters.size(); ++i) {
        if (assigned[i]) continue;

        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> row;
        row.push_back(clusters[i]);
        assigned[i] = true;

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

    std::sort(rows.begin(), rows.end(), [this](const auto& a, const auto& b) {
        return get_row_centroid(a).y < get_row_centroid(b).y;
    });

    return rows;
}

// 行中心点计算实现
pcl::PointXYZ CornRowDetector::get_row_centroid(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& row)
{
    pcl::PointXYZ centroid(0, 0, 0);
    int count = 0;
    for (const auto& cluster : row) {
        for (const auto& p : cluster->points) {
            centroid.x += p.x;
            centroid.y += p.y;
            centroid.z += p.z;
            count++;
        }
    }
    if (count > 0) {
        centroid.x /= count;
        centroid.y /= count;
        centroid.z /= count;
    }
    return centroid;
}

// 提取行内所有点实现
std::vector<pcl::PointXYZ> CornRowDetector::get_all_points(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& row)
{
    std::vector<pcl::PointXYZ> points;
    for (const auto& cluster : row) {
        points.insert(points.end(), cluster->points.begin(), cluster->points.end());
    }
    return points;
}

// 查找附近点实现
pcl::PointXYZ CornRowDetector::find_nearby_point(const std::vector<pcl::PointXYZ>& points, float x)
{
    if (points.empty()) return pcl::PointXYZ(x, 0, 0);

    auto it = std::lower_bound(points.begin(), points.end(), x,
        [](const pcl::PointXYZ& p, float val) { return p.x < val; });

    if (it == points.begin()) return *it;
    if (it == points.end()) return *points.rbegin();

    float d_prev = std::fabs((it-1)->x - x);
    float d_curr = std::fabs(it->x - x);
    return (d_prev < d_curr) ? *(it-1) : *it;
}

// 计算中心线实现
nav_msgs::msg::Path CornRowDetector::compute_center_line(const std::vector<std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>>& rows)
{
    nav_msgs::msg::Path path;
    if (rows.size() < 2) return path;

    const auto& left = rows[0];
    const auto& right = rows[1];

    auto left_points = get_all_points(left);
    auto right_points = get_all_points(right);

    std::sort(left_points.begin(), left_points.end(), [](const auto& a, const auto& b) {
        return a.x < b.x;
    });
    std::sort(right_points.begin(), right_points.end(), [](const auto& a, const auto& b) {
        return a.x < b.x;
    });

    if (left_points.empty() || right_points.empty()) return path;

    float min_x = std::min(left_points.front().x, right_points.front().x);
    float max_x = std::max(left_points.back().x, right_points.back().x);
    float step = (max_x - min_x) / spline_segments_;

    for (float x = min_x; x <= max_x; x += step) {
        auto l = find_nearby_point(left_points, x);
        auto r = find_nearby_point(right_points, x);

        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x = (l.x + r.x) / 2.0f;
        pose.pose.position.y = (l.y + r.y) / 2.0f;
        pose.pose.position.z = 0.0f;

        tf2::Quaternion q;
        q.setRPY(0, 0, 0);
        pose.pose.orientation = tf2::toMsg(q);

        path.poses.push_back(pose);
    }

    return path;
}

// 判断跳变实现
bool CornRowDetector::is_jump(const geometry_msgs::msg::PoseStamped& prev, const geometry_msgs::msg::PoseStamped& curr)
{
    float dx = curr.pose.position.x - prev.pose.position.x;
    float dy = curr.pose.position.y - prev.pose.position.y;
    return std::hypot(dx, dy) > max_jump_distance_;
}

// 异常值剔除实现
// 异常值剔除实现 - 修复未使用参数警告
std::vector<geometry_msgs::msg::PoseStamped> CornRowDetector::remove_outliers(
    const std::vector<geometry_msgs::msg::PoseStamped>& points)
{
    if (points.size() < 3) return points;

    // 计算Y方向均值和标准差
    std::vector<float> ys;
    for (const auto& p : points) ys.push_back(p.pose.position.y);
    float mean = std::accumulate(ys.begin(), ys.end(), 0.0f) / ys.size();
    
    // 修复未使用参数警告，用匿名参数代替命名参数
    float sq_sum = std::inner_product(ys.begin(), ys.end(), ys.begin(), 0.0f,
        [](float a, float b) { return a + b; },
        [mean](float y, float) { return (y - mean) * (y - mean); });  // 移除参数名
    float std_dev = std::sqrt(sq_sum / ys.size());

    // 保留在mean ± outlier_std_threshold_*std_dev范围内的点
    std::vector<geometry_msgs::msg::PoseStamped> filtered;
    for (const auto& p : points) {
        if (std::fabs(p.pose.position.y - mean) <= outlier_std_threshold_ * std_dev) {
            filtered.push_back(p);
        }
    }

    return filtered.empty() ? points : filtered;
}

// 空间平滑实现
std::vector<geometry_msgs::msg::PoseStamped> CornRowDetector::spatial_smoothing(
    const std::vector<geometry_msgs::msg::PoseStamped>& poses)
{
    if (poses.size() < 3) return poses;

    std::vector<geometry_msgs::msg::PoseStamped> smoothed = poses;
    float w = spatial_smooth_weight_;  // 中心权重
    float s = (1.0f - w) / 2.0f;       // 两侧权重

    for (size_t i = 1; i < poses.size() - 1; ++i) {
        // 加权平均平滑
        smoothed[i].pose.position.x = s * poses[i-1].pose.position.x + 
                                      w * poses[i].pose.position.x + 
                                      s * poses[i+1].pose.position.x;

        smoothed[i].pose.position.y = s * poses[i-1].pose.position.y + 
                                      w * poses[i].pose.position.y + 
                                      s * poses[i+1].pose.position.y;
    }

    return smoothed;
}

// 时间平滑实现（修正类型比较）
std::vector<geometry_msgs::msg::PoseStamped> CornRowDetector::temporal_smoothing(
    const std::vector<geometry_msgs::msg::PoseStamped>& current_poses)
{
    if (current_poses.empty()) return current_poses;

    // 维护窗口大小（使用int避免类型警告）
    while (static_cast<int>(path_buffer_.size()) > time_window_size_) {  // 显式转换为int
        path_buffer_.pop_front();
    }
    path_buffer_.push_back(current_poses);

    // 窗口未满时返回当前帧
    if (static_cast<int>(path_buffer_.size()) < time_window_size_) {  // 显式转换为int
        return current_poses;
    }

    // 多帧加权平均
    std::vector<geometry_msgs::msg::PoseStamped> smoothed = current_poses;
    size_t min_size = current_poses.size();
    for (const auto& p : path_buffer_) {
        min_size = std::min(min_size, p.size());
    }

    for (size_t i = 0; i < min_size; ++i) {
        float sum_x = 0, sum_y = 0;
        int count = 0;

        // 对窗口内所有帧的对应点求平均
        for (const auto& p : path_buffer_) {
            // 跳过跳变点
            if (count > 0 && is_jump(p[i], path_buffer_[count-1][i])) {
                continue;
            }
            sum_x += p[i].pose.position.x;
            sum_y += p[i].pose.position.y;
            count++;
        }

        if (count > 0) {
            smoothed[i].pose.position.x = sum_x / count;
            smoothed[i].pose.position.y = sum_y / count;
        }
    }

    return smoothed;
}

// 高级平滑主函数实现
nav_msgs::msg::Path CornRowDetector::advanced_smoothing(const nav_msgs::msg::Path& raw_path)
{
    nav_msgs::msg::Path smooth_path = raw_path;
    if (raw_path.poses.empty()) return smooth_path;

    // 1. 先剔除异常值
    auto filtered = remove_outliers(raw_path.poses);
    if (filtered.empty()) return smooth_path;

    // 2. 空间平滑（单帧内）
    auto spatial_smoothed = spatial_smoothing(filtered);

    // 3. 时间平滑（多帧间）
    auto temporal_smoothed = temporal_smoothing(spatial_smoothed);

    smooth_path.poses = temporal_smoothed;
    return smooth_path;
}
    