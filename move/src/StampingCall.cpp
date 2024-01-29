#include <boat.h>
#include <gps2enu.h>
#include <geometry_msgs/msg/pose_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>


namespace move
{

// main node
class StampingCall : public rclcpp::Node
{
public:
  StampingCall() : Node("StampingCall"), boat{this, &enu}
  {
    set_parameter(rclcpp::Parameter("use_sim_time", true));

    static auto gps_sub = create_subscription<NavSatFix>("/wamv/sensors/gps/gps/fix", 5, [this](NavSatFix::UniquePtr msg)
    {std::cout<<"updating pose"<<std::endl;
            boat.updatePose(*msg);});

    static auto imu_sub = create_subscription<Imu>("/wamv/sensors/imu/imu/data", 1, [this](Imu::UniquePtr msg)
    {boat.updateImu(*msg);});

    static auto buoy_sub = create_subscription<ParamVec>("/wamv/sensors/acoustics/receiver/range_bearing", 1, [this](ParamVec::UniquePtr msg)
    {boat.updateBuoy(*msg);});
  }

private:
  toENU enu;
  Boat boat;
};
}


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<move::StampingCall>());
    rclcpp::shutdown();
    return 0;
}

