#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/filters/extract_indices.h"
#include <vector>

#include <pcl/search/kdtree.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <pcl/segmentation/extract_clusters.h>

using PointCloudXYZ = pcl::PointCloud<pcl::PointXYZ>;
using PointCloudXYZPtr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

class PCLLearning : public rclcpp::Node
{
private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr fillter_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    double z_min_, z_max_;
    double voxel_size_;
    double cluster_tolerance_;
    int min_cluster_size_;
    int max_cluster_size_;

public:
    PCLLearning() : Node("pcl_filter_learning"), z_min_(0.1), z_max_(1.5), voxel_size_(0.1)
    {
        // 声明并获取参数
        this->declare_parameter("z_min", 0.0);
        this->declare_parameter("z_max", 1.5);
        this->declare_parameter("voxel_size", 0.01);

        // 使用更现代的参数获取方式
        z_min_ = this->get_parameter("z_min").as_double();
        z_max_ = this->get_parameter("z_max").as_double();
        voxel_size_ = this->get_parameter("voxel_size").as_double();

        // 声明参数 (调整聚类参数以适应更细的玉米秸秆)
        this->declare_parameter("cluster_tolerance", 0.01); // 聚类距离容差 (从0.02降低到0.01米)
        this->declare_parameter("min_cluster_size", 10);    // 最小聚类点数 (从20降低到10)
        this->declare_parameter("max_cluster_size", 5000);  // 最大聚类点数

        // 获取参数值
        cluster_tolerance_ = this->get_parameter("cluster_tolerance").as_double();
        min_cluster_size_ = this->get_parameter("min_cluster_size").as_int();
        max_cluster_size_ = this->get_parameter("max_cluster_size").as_int();

        // 先创建发布者和订阅者
        fillter_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_points", 10);
        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/mid360_PointCloud2", 10, std::bind(&PCLLearning::cloud_callback, this, std::placeholders::_1));
        // 创建发布者，发布聚类中心或边界框的Marker，便于在Rviz中可视化
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/cluster_markers", 10);
        RCLCPP_INFO(this->get_logger(), "PCL Filter Learning Node Started");
    }

private:
    /** \brief 点云滤波和降采样
     * \param[in] cloud pcl格式的点云数据
     * \return 滤波和降采样后的点云数据，pcl格式
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr filter_dowmsample(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
    {
        // Z轴直通滤波
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(z_min_, z_max_);
        pass.filter(*cloud_filtered);

        // 体素降采样
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
        voxel_grid.setInputCloud(cloud_filtered);
        voxel_grid.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
        voxel_grid.filter(*cloud_downsampled);

        return cloud_downsampled;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_segmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_no_ground(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(300);
        seg.setDistanceThreshold(0.04);
        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);

        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud);
        extract.setIndices(inliers);
        extract.setNegative(true); // 保留非地面点
        extract.filter(*cloud_no_ground);

        return cloud_no_ground;
    }

    // 提取玉米秸秆圆柱体主干（忽略叶子）
    std::vector<PointCloudXYZPtr> extract_corn_stalk_cylinders(
        const PointCloudXYZPtr &non_ground_cloud)
    {
        std::vector<PointCloudXYZPtr> stalk_cylinders;

        // 校验输入点云
        if (!non_ground_cloud || non_ground_cloud->empty())
        {
            RCLCPP_WARN(this->get_logger(), "输入点云为空或无效");
            return stalk_cylinders;
        }

        PointCloudXYZPtr remaining_cloud(new PointCloudXYZ(*non_ground_cloud));

        while (remaining_cloud->size() > 50)
        { // 确保有足够的点进行检测
            RCLCPP_WARN(this->get_logger(), "点云数量大于50，开始检测");
            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
            RCLCPP_WARN(this->get_logger(), "开始圆柱体检测");

            pcl::SACSegmentation<pcl::PointXYZ> seg;
            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_CYLINDER); // 圆柱体模型
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setMaxIterations(1500);
            seg.setDistanceThreshold(0.04f);

            // 关键修复：设置圆柱体主轴方向（玉米秸秆垂直生长，沿Z轴）
            Eigen::Vector3f axis = Eigen::Vector3f::UnitZ(); // Z轴方向
            seg.setAxis(axis);
            seg.setEpsAngle(0.2f); // 允许±11.5度的偏差（弧度）

            // 设置半径范围（玉米秸秆直径）
            seg.setRadiusLimits(0.005f, 0.08f); // 3-16cm (将最小直径从4cm降低到3cm以增加容差)

            seg.setInputCloud(remaining_cloud);

            // 检测并处理可能的错误
            try
            {
                seg.segment(*inliers, *coefficients);
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(this->get_logger(), "圆柱体检测失败: %s", e.what());
                break;
            }

            // 检查是否检测到足够的内点 (降低阈值以适应更细的玉米秸秆)
            if (inliers->indices.size() < 15)
            {
                RCLCPP_DEBUG(this->get_logger(), "内点数量不足，停止检测");
                break;
            }

            // 提取圆柱体点云
            PointCloudXYZPtr cylinder(new PointCloudXYZ);
            pcl::ExtractIndices<pcl::PointXYZ> extract;
            extract.setInputCloud(remaining_cloud);
            extract.setIndices(inliers);
            extract.setNegative(false);
            extract.filter(*cylinder);

            // 过滤符合高度要求的秸秆
            if (!cylinder->empty())
            {
                auto [min_z, max_z] = std::minmax_element(
                    cylinder->points.begin(), cylinder->points.end(),
                    [](const pcl::PointXYZ &a, const pcl::PointXYZ &b)
                    {
                        return a.z < b.z;
                    });
                float height = max_z->z - min_z->z;

                if (height >= 0.3f)
                { // 高度至少30cm (降低高度阈值以适应更矮的玉米秸秆)
                    stalk_cylinders.push_back(cylinder);
                    RCLCPP_DEBUG(this->get_logger(), "检测到玉米秸秆: 高度=%.2fm, 点数=%zu",
                                 height, cylinder->size());
                }
            }

            // 移除已检测的点，避免重复
            extract.setNegative(true);
            PointCloudXYZPtr temp(new PointCloudXYZ);
            extract.filter(*temp);
            remaining_cloud.swap(temp);
        }

        return stalk_cylinders;
    }

    // 新增函数：提取玉米秸秆直线（针对较细的玉米秸秆）
    std::vector<PointCloudXYZPtr> extract_corn_stalk_lines(
        const PointCloudXYZPtr &non_ground_cloud)
    {
        std::vector<PointCloudXYZPtr> stalk_lines;

        // 校验输入点云
        if (!non_ground_cloud || non_ground_cloud->empty())
        {
            RCLCPP_WARN(this->get_logger(), "输入点云为空或无效");
            return stalk_lines;
        }

        PointCloudXYZPtr remaining_cloud(new PointCloudXYZ(*non_ground_cloud));

        // 限制循环次数以避免过度检测
        int max_iterations = 10;
        int iteration_count = 0;

        while (remaining_cloud->size() > 30 && iteration_count < max_iterations)
        { // 确保有足够的点进行检测
            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

            // 使用RANSAC拟合直线
            pcl::SACSegmentation<pcl::PointXYZ> seg;
            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_LINE); // 直线模型
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setMaxIterations(1000);
            seg.setDistanceThreshold(0.015f); // 更严格的距离阈值，适合细小物体

            // 设置直线方向（玉米秸秆垂直生长，沿Z轴）
            Eigen::Vector3f axis = Eigen::Vector3f::UnitZ(); // Z轴方向
            seg.setAxis(axis);
            seg.setEpsAngle(0.2f); // 允许±11.5度的偏差（弧度），更严格的约束

            seg.setInputCloud(remaining_cloud);

            // 检测并处理可能的错误
            try
            {
                seg.segment(*inliers, *coefficients);
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(this->get_logger(), "直线检测失败: %s", e.what());
                break;
            }

            // 检查是否检测到足够的内点 (针对细小玉米秸秆优化)
            if (inliers->indices.size() < 15)
            {
                RCLCPP_DEBUG(this->get_logger(), "内点数量不足，停止检测");
                break;
            }

            // 提取直线点云
            PointCloudXYZPtr line(new PointCloudXYZ);
            pcl::ExtractIndices<pcl::PointXYZ> extract;
            extract.setInputCloud(remaining_cloud);
            extract.setIndices(inliers);
            extract.setNegative(false);
            extract.filter(*line);

            // 过滤符合高度和点数要求的秸秆
            if (!line->empty())
            {
                auto [min_z, max_z] = std::minmax_element(
                    line->points.begin(), line->points.end(),
                    [](const pcl::PointXYZ &a, const pcl::PointXYZ &b)
                    {
                        return a.z < b.z;
                    });
                float height = max_z->z - min_z->z;

                // 检查高度和点数，专门针对3-6cm直径的玉米秸秆
                if (height >= 0.15f && line->size() >= 15 && line->size() <= 100)
                { // 高度至少15cm，点数在合理范围内避免检测到大平面
                    stalk_lines.push_back(line);
                    RCLCPP_DEBUG(this->get_logger(), "检测到玉米秸秆（直线）: 高度=%.2fm, 点数=%zu",
                                 height, line->size());
                }
            }

            // 移除已检测的点，避免重复
            extract.setNegative(true);
            PointCloudXYZPtr temp(new PointCloudXYZ);
            extract.filter(*temp);
            remaining_cloud.swap(temp);
            
            iteration_count++;
        }

        return stalk_lines;
    }

    // 预处理点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr preprocess_cloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);

        // 高度过滤
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(input);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(0.4, 2.0);
        pass.filter(*filtered);

        return filtered;
    }

    // 合并多个点云为一个
    pcl::PointCloud<pcl::PointXYZ>::Ptr merge_clouds(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &clouds)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr merged(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto &cloud : clouds)
        {
            *merged += *cloud;
        }
        return merged;
    }

    void new_test(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // 1. 将ROS2 PointCloud2消息转换为PCL点云格式
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        if (cloud->empty())
        {
            RCLCPP_WARN(this->get_logger(), "Received an empty cloud, skipping processing.");
            return;
        }

        // 2. 构建KD-Tree用于快速近邻搜索
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud);

        // 3. 执行欧几里得聚类提取
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(cluster_tolerance_); // 设置聚类距离容差
        ec.setMinClusterSize(min_cluster_size_);    // 设置最小聚类点数
        ec.setMaxClusterSize(max_cluster_size_);    // 设置最大聚类点数
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);

        RCLCPP_INFO(this->get_logger(), "Number of clusters found: %zu", cluster_indices.size());

        // 4. 处理聚类结果
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
        visualization_msgs::msg::MarkerArray marker_array;
        int marker_id = 0;

        // 为每个聚类创建Marker并提取点云（可选）
        for (const auto &indices : cluster_indices)
        {
            // 提取单个聚类的点云
            pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            for (const auto &idx : indices.indices)
            {
                cluster_cloud->push_back((*cloud)[idx]);
            }
            clusters.push_back(cluster_cloud);

            // 计算聚类的中心点（简单起见，使用第一个点，或可计算质心）
            // 更准确的做法是计算整个聚类的质心
            float cx = 0, cy = 0, cz = 0;
            for (const auto &point : *cluster_cloud)
            {
                cx += point.x;
                cy += point.y;
                cz += point.z;
            }
            cx /= cluster_cloud->size();
            cy /= cluster_cloud->size();
            cz /= cluster_cloud->size();

            // 创建可视化Marker（例如球体）
            visualization_msgs::msg::Marker marker;
            marker.header = msg->header;
            marker.ns = "corn_stalk_clusters";
            marker.id = marker_id++;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = cx;
            marker.pose.position.y = cy;
            marker.pose.position.z = cz;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.1; // Marker大小
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;                                  // 不透明度
            marker.lifetime = rclcpp::Duration::from_seconds(0.2); // 短暂显示

            marker_array.markers.push_back(marker);
        }

        // 5. 发布聚类结果（这里选择发布所有聚类点云的合并，实际可按需调整）
        if (!clusters.empty())
        {
            pcl::PointCloud<pcl::PointXYZ> merged_cloud;
            for (const auto &cluster : clusters)
            {
                merged_cloud += *cluster;
            }
            sensor_msgs::msg::PointCloud2 output_msg;
            pcl::toROSMsg(merged_cloud, output_msg);
            output_msg.header = msg->header;
            fillter_pub_->publish(output_msg);
        }

        // 发布可视化Marker
        marker_pub_->publish(marker_array);
    };

    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // 转换为PCL点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        // 预处理点云，进行滤波和降采样
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered = filter_dowmsample(cloud);

        // 平面分割，去除地面干扰
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_segmentation = ground_segmentation(cloud_filtered);

        // // 2. 预处理和提取玉米秸秆
        // pcl::PointCloud<pcl::PointXYZ>::Ptr preprocessed = preprocess_cloud(cloud_filtered);
        // std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> stalks = extract_corn_stalk_cylinders(preprocessed);

        // if (stalks.empty())
        // {
        //     RCLCPP_INFO(this->get_logger(), "未检测到玉米秸秆");
        //     // return;
        // }

        // // 3. 合并所有秸秆点云
        // pcl::PointCloud<pcl::PointXYZ>::Ptr merged_stalks = merge_clouds(stalks);
        // 转换回sensor_msgs::msg::PointCloud2格式并发布


        // sensor_msgs::msg::PointCloud2::SharedPtr after_processed_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
        // pcl::toROSMsg(*cloud_segmentation, *after_processed_msg);
        // after_processed_msg->header = msg->header; // 保持原始时间戳和坐标帧信息

        // new_test(after_processed_msg);
// 

        std::vector<PointCloudXYZPtr> corn_lines = extract_corn_stalk_lines(cloud_segmentation);
        
        // 合并所有直线点云
        PointCloudXYZPtr merged_lines(new PointCloudXYZ);
        for (const auto& line_cloud : corn_lines) {
            *merged_lines += *line_cloud;
        }
        
        // 转换为ROS消息并发布
        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(*merged_lines, output_msg);
        output_msg.header = msg->header;
        fillter_pub_->publish(output_msg);

        // RCLCPP_INFO(this->get_logger(), "Published filtered point cloud with %ld points", cloud_filtered->size());
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PCLLearning>());
    rclcpp::shutdown();
    return 0;
}