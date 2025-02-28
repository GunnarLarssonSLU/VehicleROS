#ifndef MY_VEHICLE__VEHICLE_NODE_HPP_
#define MY_VEHICLE__VEHICLE_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>  // For demonstration
#include <std_msgs/msg/float64.hpp>  // Updated to use Float64 for continuous values
#include "my_vehicle/srv/set_vehicle_id.hpp"  // Include the custom service header
#include "my_vehicle/msg/direct_control.hpp"  // Include the custom message
#include "my_vehicle/msg/hydraulics.hpp"  // Include the custom message
#include <QByteArray>
#include <QLocalSocket>
#include <QLocalServer>

// If using your custom services:
#include "my_vehicle/srv/get_position.hpp"
#include "my_vehicle/srv/get_battery_state.hpp"


namespace my_vehicle
{

class VehicleNode : public rclcpp::Node
{

public:
  VehicleNode();
  ~VehicleNode();

    
private:
    uint8_t vehicle_id_;  // Member variable to store the vehicle ID
    
  // Subscriptions
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr steering_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr velocity_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr terminal_sub_;
  rclcpp::Subscription<my_vehicle::msg::Hydraulics>::SharedPtr hydraulics_sub_;
  rclcpp::Subscription<my_vehicle::msg::DirectControl>::SharedPtr directcontrol_sub_;

  // Services
  rclcpp::Service<my_vehicle::srv::GetPosition>::SharedPtr get_position_srv_;
  rclcpp::Service<my_vehicle::srv::GetBatteryState>::SharedPtr get_battery_state_srv_;
  rclcpp::Service<my_vehicle::srv::SetVehicleId>::SharedPtr set_vehicle_id_service_;


  // Callbacks
  void onDirectControlCmd(const my_vehicle::msg::DirectControl::SharedPtr msg);
  void onHydraulicsCmd(const my_vehicle::msg::Hydraulics::SharedPtr msg);
  void onSteeringCmd(const std_msgs::msg::Float64::SharedPtr msg);
  void onVelocityCmd(const std_msgs::msg::Float64::SharedPtr msg);
  void onTerminalCmd(const std_msgs::msg::String::SharedPtr msg);

  // For internal use
  void sendCommand(const QByteArray &data);
  QByteArray standardMessageStart(int id);

  // Service callbacks
  void onGetPosition(
    const std::shared_ptr<my_vehicle::srv::GetPosition::Request> request,
    std::shared_ptr<my_vehicle::srv::GetPosition::Response> response);

  void onGetBatteryState(
    const std::shared_ptr<my_vehicle::srv::GetBatteryState::Request> request,
    std::shared_ptr<my_vehicle::srv::GetBatteryState::Response> response);

    // Function to set the vehicle ID
 void setVehicleIdCallback(
    const std::shared_ptr<my_vehicle::srv::SetVehicleId::Request> request,
    std::shared_ptr<my_vehicle::srv::SetVehicleId::Response> response);



    // ROS 2 terminal output publisher
    void setupTerminalServer();
    void processTerminalConnections();
    void processClientSocket(QLocalSocket *client_connection);

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr terminal_publisher_;
    QLocalServer *terminal_server_;

};

}  // namespace my_vehicle

#endif  // MY_VEHICLE__VEHICLE_NODE_HPP_

