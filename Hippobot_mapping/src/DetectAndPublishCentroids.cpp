#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <iostream>
#include <my_robot_interfaces/msg/centroid_with_radius.hpp>
#include <my_robot_interfaces/msg/centroid_array_with_radius.hpp>

class PointCloudProcessor : public rclcpp::Node {
public:
    PointCloudProcessor() : Node("point_cloud_processor") {
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/wamv/sensors/lidars/lidar_wamv_sensor/points", 10, std::bind(&PointCloudProcessor::pointCloudCallback, this, std::placeholders::_1));

        subscription_me = this->create_subscription<geometry_msgs::msg::Pose>(
            "/boat_pose", 10, std::bind(&PointCloudProcessor::BoatPoseCallback, this, std::placeholders::_1));
        // Create a publisher for PoseArray
        centroid_publisher_ = this->create_publisher<my_robot_interfaces::msg::CentroidArrayWithRadius>("centroids", 10);

        set_parameter(rclcpp::Parameter("use_sim_time",true));

    }

private:


    float euclideanDistance(const pcl::PointXYZ& point, const Eigen::Vector4f& centroid) {
        float dx = point.x - centroid[0];
        float dy = point.y - centroid[1];
        float dz = point.z - centroid[2];
        return sqrt(dx * dx + dy * dy + dz * dz);
    }

    void BoatPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg ){
        boat_X = msg->position.x;
        boat_Y = msg->position.y;
    }

    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        // Filter based on z-coordinate
        filterPointCloud(cloud);

        // Extract clusters
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
        extractClusters(cloud, clusters);

        // Calculate centroids and publish as PoseArray
        calculateCentroidsAndPublish(clusters);
    }

    void filterPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(-0.7, FLT_MAX); 
        pass.filter(*cloud);
    }

    void extractClusters(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                         std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clusters) {
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(10.);  
        ec.setMinClusterSize(2);        
        ec.setInputCloud(cloud);

        std::vector<pcl::PointIndices> cluster_indices;
        ec.extract(cluster_indices);

        for (const auto& indices : cluster_indices) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
            for (const auto& index : indices.indices) {
                cluster->points.push_back(cloud->points[index]);
            }
            clusters.push_back(cluster);
        }
    }

    void calculateCentroidsAndPublish(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clusters) {

        my_robot_interfaces::msg::CentroidArrayWithRadius CentroidArrayWithRadius;

        for (size_t i = 0; i < clusters.size(); ++i) {
            my_robot_interfaces::msg::CentroidWithRadius CentroidWithRadius;
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*clusters[i], centroid);


            CentroidWithRadius.position.x = boat_X + centroid[0];
            CentroidWithRadius.position.y = boat_Y + centroid[1];
            CentroidWithRadius.position.z = centroid[2];

            float max_distance = 0.0f;
            for (const auto& point : clusters[i]->points) {
                float distance = euclideanDistance(point, centroid);
                if (distance > max_distance) {
                max_distance = distance;
                }
            }

            CentroidWithRadius.radius = max_distance;
            CentroidArrayWithRadius.centroids.push_back(CentroidWithRadius);
        }

        centroid_publisher_->publish(CentroidArrayWithRadius);
    }

    float boat_X;
    float boat_Y;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription_me;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<my_robot_interfaces::msg::CentroidArrayWithRadius>::SharedPtr centroid_publisher_;
};



int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudProcessor>());
    rclcpp::shutdown();
    return 0;
}
