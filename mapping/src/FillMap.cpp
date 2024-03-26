#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <map_msgs/msg/occupancy_grid_update.hpp>
#include <iostream>
#include <ctime>
#include <my_robot_interfaces/msg/centroid_with_radius.hpp>
#include <my_robot_interfaces/msg/centroid_array_with_radius.hpp>
//#include <tf2_ros/static_transform_broadcaster.h>
//#include <tf2/LinearMath/Quaternion.h>

// TO DO : Make sure the update topic doesn't publish out of bounds and reduce initial map publishing time ( ligne 36-41 )

class Mapping : public rclcpp::Node {
public:
    Mapping() :
        Node("onfailamap"){

            subscription_ = this->create_subscription<my_robot_interfaces::msg::CentroidArrayWithRadius>(
                "centroids", 10, std::bind(&Mapping::UpdateCentroids, this, std::placeholders::_1));
            subscription2_= this->create_subscription<nav_msgs::msg::Odometry>(
                        "wamv/odom", 10, std::bind(&Mapping::BoatPoseCallback, this, std::placeholders::_1));
            
            
            MapPublisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("WorldMap", 10); //TO DO use transient local instead of volatile 
            MapUpdatePublisher_ = this->create_publisher<map_msgs::msg::OccupancyGridUpdate>("WorldMap_updates", 10);
            
            // Create a publisher for PoseArray
            set_parameter(rclcpp::Parameter("use_sim_time",true));
            
                    }
            void init(){
              // Create the map and publish it in a loop to make sure Rviz has time to receive it
              auto start = std::chrono::system_clock::now();
              while (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - start).count() < 10)
              {
                MapCreate();
              }              
                    }
            

private:


    void MapCreate()
    {
        std::cout << "map creation"<<std::endl;
        map_.header.stamp = this->now();
        map_.header.frame_id = "world";
        map_.info.width = 1000;
        map_.info.height = 1000;
        map_.info.resolution = 1;
        map_.info.origin.position.x = -500;
        map_.info.origin.position.y = -500;

        map_.data.resize(map_.info.width*map_.info.height,0);
        MapPublisher_->publish(map_);
    }


    void BoatPoseCallback(const nav_msgs::msg::Odometry::SharedPtr msg ){
      posX = msg->pose.pose.position.x;
      posY = msg->pose.pose.position.y;
    }



    void UpdateCentroids(const my_robot_interfaces::msg::CentroidArrayWithRadius::SharedPtr CentroidsWithRadius)
      {

        std::cout << "traitement centroides";
        if (count == refresh)
        {
            UpdateMap(posX,posY,100,0);
            //UpdateMap(posX,posY,1,-1);
            count = 0;
        }
        else
        {
            count=count+1;
        }


        for (const my_robot_interfaces::msg::CentroidWithRadius& centroid : CentroidsWithRadius->centroids)
        {
          auto center = centroid.position;
          auto radius = centroid.radius;
          int x = (int)center.x;
          int y = (int)center.y;
          int r = (int)radius;
          if (-100<x-posX && x-posX<100 && -100<y-posY && y-posY<100)
          {
            UpdateMap(x,y,r,1);
          }
        }
      }




    void UpdateMap(const int x, const int y, const int r, const int val)
      {
          std::cout << "update map"<<std::endl;
          map_msgs::msg::OccupancyGridUpdate UpdateMap;
          UpdateMap.header.stamp = this->now();
          UpdateMap.header.frame_id = "world";
          UpdateMap.x = 500+x-r;
          UpdateMap.y = 500+y-r;
          UpdateMap.width = 2*r+1;
          UpdateMap.height = 2*r+1;
          UpdateMap.data.resize(UpdateMap.width*UpdateMap.height,val);

          MapUpdatePublisher_->publish(UpdateMap);
      }




      rclcpp::Subscription<my_robot_interfaces::msg::CentroidArrayWithRadius>::SharedPtr subscription_;
      rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription2_;
      rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr MapPublisher_;
      rclcpp::Publisher<map_msgs::msg::OccupancyGridUpdate>::SharedPtr MapUpdatePublisher_;
      nav_msgs::msg::OccupancyGrid map_;


      int posX = 0;
      int posY = 0;

      int count = 0;
      int refresh = 10;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto processor = std::make_shared<Mapping>();
    processor->init();
    rclcpp::spin(processor);
    rclcpp::shutdown();
    return 0;
}
