#include "my_vehicle/vehicle_node.hpp"
#include <rclcpp/rclcpp.hpp>


// "main" function
#include <memory>

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<my_vehicle::VehicleNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

