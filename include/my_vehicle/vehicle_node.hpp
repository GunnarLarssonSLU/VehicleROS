#ifndef MY_VEHICLE__VEHICLE_NODE_HPP_
#define MY_VEHICLE__VEHICLE_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>  // For demonstration
#include <std_msgs/msg/float64.hpp>  // Updated to use Float64 for continuous values
#include <QByteArray>

// If using your custom services:
#include "my_vehicle/srv/get_position.hpp"
#include "my_vehicle/srv/get_battery_state.hpp"


namespace my_vehicle
{

class VehicleNode : public rclcpp::Node
{
public:
  VehicleNode();
  ~VehicleNode() = default;

private:
  // Subscriptions
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr steering_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr velocity_sub_;

  // Services
  rclcpp::Service<my_vehicle::srv::GetPosition>::SharedPtr get_position_srv_;
  rclcpp::Service<my_vehicle::srv::GetBatteryState>::SharedPtr get_battery_state_srv_;

  // Callbacks
  void onSteeringCmd(const std_msgs::msg::Float64::SharedPtr msg);
  void onVelocityCmd(const std_msgs::msg::String::SharedPtr msg);
  void sendCommand(const QByteArray &data);

  // Service callbacks
  void onGetPosition(
    const std::shared_ptr<my_vehicle::srv::GetPosition::Request> request,
    std::shared_ptr<my_vehicle::srv::GetPosition::Response> response);

  void onGetBatteryState(
    const std::shared_ptr<my_vehicle::srv::GetBatteryState::Request> request,
    std::shared_ptr<my_vehicle::srv::GetBatteryState::Response> response);
};

}  // namespace my_vehicle

#endif  // MY_VEHICLE__VEHICLE_NODE_HPP_

