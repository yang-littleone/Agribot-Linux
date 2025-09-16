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
#include <pcl/filters/statistical_outlier_removal.h>
#include <limits> // 添加limits头文件以支持std::numeric_limits
#include <pcl/octree/octree_search.h>

using PointCloudXYZ = pcl::PointCloud<pcl::PointXYZ>;
using PointCloudXYZPtr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

class CenterLineExtract : public rclcpp::Node
{
private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr fillter_pub_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr last_cloud_;
    double y_min_, y_max_, z_min_, z_max_, voxel_size_;

public:
    CenterLineExtract(/* args */) : Node("corn_centerline")
    {
        // 声明并获取参数
        this->declare_parameter("y_min", -1.0);
        this->declare_parameter("y_max", 1.0);
        this->declare_parameter("z_min", 0.0);
        this->declare_parameter("z_max", 1.5);
        this->declare_parameter("voxel_size", 0.02);

        // 使用更现代的参数获取方式
        z_min_ = this->get_parameter("z_min").as_double();
        z_max_ = this->get_parameter("z_max").as_double();
        voxel_size_ = this->get_parameter("voxel_size").as_double();

        // 先创建发布者和订阅者
        fillter_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_points", 10);
        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/Laser_map", 10, std::bind(&CenterLineExtract::cloud_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "PCL Filter Learning Node Started");
    }

private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr subFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud, pcl::IndicesPtr indices, float cell_x, float cell_y, float cell_z)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_pointcloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud(pointcloud);
        sor.setIndices(indices);
        sor.setLeafSize(cell_x, cell_y, cell_z);
        sor.filter(*filtered_pointcloud); // No problem :)
        return filtered_pointcloud;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr OctFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn, float cell_x, float cell_y, float cell_z)
    {
        pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(10240); // // Octree resolution - side length of octree voxels
        octree.setInputCloud(cloudIn);
        octree.addPointsFromInputCloud();
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (auto it = octree.leaf_depth_begin(); it != octree.leaf_depth_end(); ++it)
        {

            pcl::IndicesPtr indexVector(new std::vector<int>);
            pcl::octree::OctreeContainerPointIndices &container = it.getLeafContainer();

            container.getPointIndices(*indexVector);
            *filtered_cloud += *subFilter(cloudIn, indexVector, cell_x, cell_y, cell_z);
        }
        return filtered_cloud;
    }
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
        // 假设玉米行大致沿Y方向，设置X范围保留两行之间的区域
        pass.setFilterFieldName("y");
        pass.setFilterLimits(y_min_, y_max_); // 根据实际情况调整
        pass.setFilterFieldName("z");
        pass.setFilterLimits(z_min_, z_max_);
        pass.filter(*cloud_filtered);

        // // 检查点云是否为空
        // if (cloud_filtered->empty()) {
        //     RCLCPP_WARN(this->get_logger(), "Filtered cloud is empty!");
        //     return cloud_filtered;
        // }

        // 体素降采样
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZ>);
        // pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
        // voxel_grid.setInputCloud(cloud_filtered);
        // voxel_grid.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
        // voxel_grid.filter(*cloud_downsampled);
        // RCLCPP_INFO(this->get_logger(), "After pass through filter: %ld points", cloud_downsampled->size());

        cloud_downsampled = OctFilter(cloud_filtered, voxel_size_, voxel_size_, voxel_size_);
        if (cloud_downsampled->size() > 15000)
            return last_cloud_;
        RCLCPP_INFO(this->get_logger(), "After pass through filter: %ld points", cloud_downsampled->size());
        last_cloud_ = cloud_downsampled;
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

        // 检查是否找到了平面，并验证其法向量是否接近Z轴方向
        if (inliers->indices.size() > 0 && coefficients->values.size() == 4)
        {
            // 平面方程: ax + by + cz + d = 0
            // 法向量: (a, b, c)
            float a = coefficients->values[0];
            float b = coefficients->values[1];
            float c = coefficients->values[2];

            // 计算法向量的模长
            float norm = std::sqrt(a * a + b * b + c * c);

            if (norm > 0)
            {
                // 归一化法向量
                float normalized_c = c / norm;

                // 检查法向量是否接近Z轴方向(垂直向上)
                // 我们接受一定角度的偏差，例如与Z轴夹角小于30度(cos(30°) ≈ 0.866)
                if (normalized_c > 0.866)
                {
                    // 这是一个接近Z轴向上的平面，很可能是地面
                    pcl::ExtractIndices<pcl::PointXYZ> extract;
                    extract.setInputCloud(cloud);
                    extract.setIndices(inliers);
                    extract.setNegative(true); // 保留非地面点
                    extract.filter(*cloud_no_ground);

                    return cloud_no_ground;
                }
            }
        }

        // 如果没有找到合适的地面平面，则返回原始点云
        return cloud;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr segmentStalks(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
    {
        // 创建KD树用于快速近邻搜索
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud);

        // 欧式聚类对象
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(0.03); // 增加聚类距离阈值到2cm，更适合检测细小农作物
        ec.setMinClusterSize(5);      // 减小最小聚类点数，以检测较小的农作物部分
        ec.setMaxClusterSize(1000);   // 减小最大聚类点数，避免大平面被聚类
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);

        RCLCPP_INFO(this->get_logger(), "Found %zu clusters", cluster_indices.size());

        // 存储所有聚类
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
        for (const auto &indices : cluster_indices)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::copyPointCloud(*cloud, indices, *cluster);
            clusters.push_back(cluster);
        }

        // 创建一个点云来存储所有识别为秸秆的点
        pcl::PointCloud<pcl::PointXYZ>::Ptr stalk_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        // 存储所有被识别为秸秆的独立植株
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> stalks;

        // 对每个未处理的聚类，尝试构建一个完整的植株
        std::vector<bool> processed_clusters(clusters.size(), false);
        for (size_t i = 0; i < clusters.size(); ++i)
        {
            if (processed_clusters[i])
                continue;

            // 检查当前聚类是否为秸秆的一部分
            if (!isStalkCluster(clusters[i]))
                continue;

            // 创建一个新的点云来存储当前植株的所有段
            pcl::PointCloud<pcl::PointXYZ>::Ptr current_stalk(new pcl::PointCloud<pcl::PointXYZ>);
            *current_stalk += *clusters[i];
            processed_clusters[i] = true;

            // 查找与当前聚类相邻的其他聚类
            for (size_t j = 0; j < clusters.size(); ++j)
            {
                if (processed_clusters[j])
                    continue;

                // 检查两个聚类是否相邻
                if (areClustersAdjacent(current_stalk, clusters[j]))
                {
                    // 检查相邻聚类是否也为秸秆的一部分
                    if (isStalkCluster(clusters[j]))
                    {
                        *current_stalk += *clusters[j];
                        processed_clusters[j] = true;
                    }
                }
            }

            // 对整个植株进行直线拟合验证
            if (isStalkCluster(current_stalk))
            {
                stalks.push_back(current_stalk);
                RCLCPP_INFO(this->get_logger(), "Identified stalk with %ld points", current_stalk->size());
            }
        }

        // 将所有识别出的植株合并到一个点云中
        for (const auto &stalk : stalks)
        {
            *stalk_cloud += *stalk;
        }

        RCLCPP_INFO(this->get_logger(), "Total stalks identified: %zu", stalks.size());
        RCLCPP_INFO(this->get_logger(), "Total stalk cloud size: %ld", stalk_cloud->size());
        return stalk_cloud;
    }

    /** \brief 判断两个聚类是否相邻
     * \param[in] cluster1 第一个聚类
     * \param[in] cluster2 第二个聚类
     * \return 如果相邻返回true，否则返回false
     */
    bool areClustersAdjacent(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster1, pcl::PointCloud<pcl::PointXYZ>::Ptr cluster2)
    {
        // 创建KD树用于快速近邻搜索
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cluster1);

        // 设置邻近距离阈值
        float distance_threshold = 0.15; // 5cm

        // 检查cluster2中的点是否与cluster1中的点邻近
        for (const auto &point : cluster2->points)
        {
            std::vector<int> indices;
            std::vector<float> squared_distances;

            if (tree->radiusSearch(point, distance_threshold, indices, squared_distances) > 0)
            {
                return true; // 找到邻近点
            }
        }

        return false;
    }

    /** \brief 判断聚类是否为秸秆点云
     * \param[in] cluster 聚类点云
     * \return 如果是秸秆点云返回true，否则返回false
     */
    bool isStalkCluster(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster)
    {
        // 点数过少或过多都不是秸秆特征
        if (cluster->size() < 5 || cluster->size() > 200)
        {
            return false;
        }

        // 使用直线拟合模型检测秸秆
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_LINE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.015); // 设置距离阈值为1.5cm
        seg.setMaxIterations(1000);      // 最大迭代次数
        seg.setInputCloud(cluster);
        seg.segment(*inliers, *coefficients);

        // 检查拟合结果
        if (inliers->indices.size() < 10)
        {
            // 内点数量不足，不是良好的直线特征
            return false;
        }

        // 计算点云在Z轴方向上的高度差
        float min_z = std::numeric_limits<float>::max();
        float max_z = std::numeric_limits<float>::lowest();

        for (const auto &idx : inliers->indices)
        {
            const auto &point = cluster->points[idx];
            if (point.z < min_z)
                min_z = point.z;
            if (point.z > max_z)
                max_z = point.z;
        }

        float height = max_z - min_z;

        // 判断是否符合秸秆特征：
        // 1. 有足够的内点支持直线模型
        // 2. 在Z轴方向上有足够的高度 (至少10cm)
        if (height > 0.1)
        {
            return true;
        }

        return false;
    }

    //  处理接收到的点云，并将处理后的点云发布出去
    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // 转换为PCL点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        // 预处理点云，进行滤波和降采样
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered = filter_dowmsample(cloud);

        // 平面分割，去除地面干扰
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_noground = ground_segmentation(cloud_filtered);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_stalks = segmentStalks(cloud_noground);
        // 转换为ROS消息并发布
        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(*cloud_stalks, output_msg);
        output_msg.header = msg->header;
        fillter_pub_->publish(output_msg);
        RCLCPP_INFO(this->get_logger(), "Published filtered point cloud with %ld points", cloud_stalks->size());
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CenterLineExtract>());
    rclcpp::shutdown();
    return 0;
}