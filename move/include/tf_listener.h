#ifndef TF_LISTENER_H
#define TF_LISTENER_H

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

using namespace geometry_msgs::msg;

namespace move
{

struct TFListener
{
  inline TFListener(rclcpp::Node *node) : tf_buffer{node->get_clock()}, tf_listener{tf_buffer}
  {}

  inline std::optional<geometry_msgs::msg::Transform> tryGetTF(const std::string &origin, const std::string &dest)
  {
    static std::string err;
    if(!tf_buffer.canTransform(origin, dest, tf2::TimePointZero, &err))
      return {};
    return tf_buffer.lookupTransform(origin, dest, tf2::TimePointZero).transform;
  }

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener;
};




}

#endif // TF_LISTENER_H
