#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <iostream>
#include <ctime>
#include <my_robot_interfaces/msg/centroid_with_radius.hpp>
#include <my_robot_interfaces/msg/centroid_array_with_radius.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

class PointCloudProcessor : public rclcpp::Node {
public:
    PointCloudProcessor() : Node("onfailamap") {
        subscription_ = this->create_subscription<my_robot_interfaces::msg::CentroidArrayWithRadius>(
            "centroids", 10, std::bind(&PointCloudProcessor::poseArrayCallback, this, std::placeholders::_1));

        // Create a publisher for PoseArray
        MapPublisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("WorldMap", 10);
        set_parameter(rclcpp::Parameter("use_sim_time",true));

    }
    // Initialize 'world' frame
    void init() {
            InitializeWorldFrame();
    }



private:
    void poseArrayCallback(const my_robot_interfaces::msg::CentroidArrayWithRadius::SharedPtr CentroidsWithRadius)
      {
        std::cout<<"entered callback"<<std::endl;
        // Traitez les données de la PoseArray ici
        nav_msgs::msg::OccupancyGrid map;
        geometry_msgs::msg::Pose origin;
        origin.position.x = -500;
        origin.position.y = -500;
        map.header.stamp = this->now();
        map.header.frame_id = "world";
        //map.info.map_load_time = this->now();
        map.info.width = 1000;
        map.info.height = 1000;
        map.info.resolution = 1;

        map.info.origin = origin;
        map.data.resize(map.info.width*map.info.height,-1);

        for (const my_robot_interfaces::msg::CentroidWithRadius& centroid : CentroidsWithRadius->centroids) {
          // Accédez aux champs de la pose (position et orientation)
          //auto position = centroid.position;
          //auto radius = centroid.radius;

          for ( unsigned int i = 0; i < map.data.size();i++)
          {
              map.data[i] = -1;
          }



          // Faites quelque chose avec les données de la pose...
        }
        MapPublisher_->publish(map);
      }

    void InitializeWorldFrame() {
            static tf2_ros::StaticTransformBroadcaster static_broadcaster(shared_from_this());
            geometry_msgs::msg::TransformStamped static_transform_stamped;

            static_transform_stamped.header.stamp = this->now();
            static_transform_stamped.header.frame_id = "wamv/wamv/base_link";
            static_transform_stamped.child_frame_id = "world";
            static_transform_stamped.transform.translation.x = 0.0;
            static_transform_stamped.transform.translation.y = 0.0;
            static_transform_stamped.transform.translation.z = 0.0;

            tf2::Quaternion quat;
            quat.setRPY(0, 0, 0);
            static_transform_stamped.transform.rotation.x = quat.x();
            static_transform_stamped.transform.rotation.y = quat.y();
            static_transform_stamped.transform.rotation.z = quat.z();
            static_transform_stamped.transform.rotation.w = quat.w();

            static_broadcaster.sendTransform(static_transform_stamped);
        }

      rclcpp::Subscription<my_robot_interfaces::msg::CentroidArrayWithRadius>::SharedPtr subscription_;
      rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr MapPublisher_;



};


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto processor = std::make_shared<PointCloudProcessor>();
    processor->init();
    rclcpp::spin(processor);
    rclcpp::shutdown();
    return 0;
}
