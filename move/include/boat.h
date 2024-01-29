#ifndef BOAT_H
#define BOAT_H

#include <rclcpp/node.hpp>
#include <gps2enu.h>
#include <tf_listener.h>
#include <covariances.h>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <ros_gz_interfaces/msg/param_vec.hpp>

using ros_gz_interfaces::msg::ParamVec;

namespace move
{

struct Boat
{
  rclcpp::Node* node;
  toENU* enu;

  // for the buoy
  tf2_ros::TransformBroadcaster br;
  TFListener listener;
  TransformStamped buoy;

  // forwarded messages
  PoseWithCovarianceStamped pose;
  Imu imu;
  rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr pose_pub;
  rclcpp::Publisher<Imu>::SharedPtr imu_pub;

  Boat(rclcpp::Node* node, toENU* enu) : node{node}, enu{enu}, br{node}, listener{node}
  {
    // init pose and imu messages
    pose.header.frame_id = "world";
    imu.header.frame_id = "wamv/wamv/imu_wamv_link/imu_wamv_sensor";

    for(auto i: {0, 7})
      pose.pose.covariance[i] = Covariances::xy;
    pose.pose.covariance[14] = Covariances::z;
    for(auto i: {21, 28, 35})
      pose.pose.covariance[i] = Covariances::rpy;

    for(auto i: {0, 4, 8})
    {
      imu.linear_acceleration_covariance[i] = Covariances::a;
      imu.angular_velocity_covariance[i] = Covariances::w;
      imu.orientation_covariance[i] = Covariances::rpy;
    }

    pose_pub = node->create_publisher<PoseWithCovarianceStamped>("/wamv/gps_pose", rclcpp::SensorDataQoS());
    imu_pub = node->create_publisher<Imu>("/wamv/imu", rclcpp::SensorDataQoS());

    buoy.header.frame_id = "wamv/wamv/receiver";
    buoy.child_frame_id = "buoy";
  }

  inline void updatePose(const NavSatFix &gps)
  {
    if(!enu->isInit())
      enu->setReference(gps);

    enu->transform(gps.latitude, gps.longitude, gps.altitude,
                   pose.pose.pose.position);

    // TODO this is the XYZ of the GPS receiver, not of the base_link
    // get offset between base and gps

    /*
    const auto offset{listener.tryGetTF("wamv/wamv/base_link", "wamv/wamv/gps_wamv_link/navsat")};
    const auto base{listener.tryGetTF("world", "wamv/wamv/base_link")};


    if(offset.has_value() && base.has_value())
    {
      pose.pose.pose.position.z -= 1.62;  // not quite
      std::cout<<"Publishing pose stamped 1"<<std::endl;
      pose.header.stamp = node->get_clock()->now();
      pose_pub->publish(pose);
    }

    */
    std::cout<<"Publishing gps_pose stamped"<<std::endl;
    pose.header.stamp = node->get_clock()->now();
    pose_pub->publish(pose);

  }

  inline void updateImu(const Imu &imu)
  {
    this->imu.orientation = imu.orientation;
    this->imu.angular_velocity = imu.angular_velocity;
    //this->imu.linear_acceleration = imu.linear_acceleration;
    this->imu.header.stamp = node->get_clock()->now();
    //std::cout<<"Publishing stamped IMU"<<std::endl;
    imu_pub->publish(this->imu);
  }

  inline void updateBuoy(const ParamVec &msg)
  {
    const double *rho{}, *theta{};
    for(const auto &param: msg.params)
    {
      if(param.name == "range")
        rho = &(param.value.double_value);
      else if(param.name == "bearing")
        theta = &(param.value.double_value);
    }

    if(rho == nullptr || theta == nullptr)
      return;

    buoy.transform.translation.x = (*rho) * cos(*theta);
    buoy.transform.translation.y = (*rho) * sin(*theta);

    buoy.header.stamp = node->get_clock()->now();

    // RELATIVE TO BOAT TO DO : MAKE IT NOT RELATIVE

    br.sendTransform(buoy);
  }

};
}

#endif // BOAT_H
