#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("pose_publisher");
    auto publisher = node->create_publisher<geometry_msgs::msg::Pose>("waypoint_pose", 10);
    rclcpp::Rate rate(10); // Publish at 10 Hz

    while (rclcpp::ok()) {
        
        auto pose_message = geometry_msgs::msg::Pose();
        pose_message.position.x = -100; // Replace with your desired x position
        pose_message.position.y = -100; // Replace with your desired y position

        publisher->publish(pose_message);
        rclcpp::spin_some(node);
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
