#include "my_vehicle/vehicle_node.hpp"
#include <QLocalSocket>

namespace my_vehicle
{

VehicleNode::VehicleNode()
: Node("vehicle_node")
{
  // Create subscriptions        
    steering_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "steering_cmd", 10,
        std::bind(&VehicleNode::onSteeringCmd, this, std::placeholders::_1));
        
  velocity_sub_ = create_subscription<std_msgs::msg::String>(
    "velocity_cmd",
    10,
    std::bind(&VehicleNode::onVelocityCmd, this, std::placeholders::_1)
  );

  // Create services
  get_position_srv_ = create_service<my_vehicle::srv::GetPosition>(
    "get_position",
    std::bind(
      &VehicleNode::onGetPosition,
      this,
      std::placeholders::_1,
      std::placeholders::_2)
  );

  get_battery_state_srv_ = create_service<my_vehicle::srv::GetBatteryState>(
    "get_battery_state",
    std::bind(
      &VehicleNode::onGetBatteryState,
      this,
      std::placeholders::_1,
      std::placeholders::_2)
  );
}

void VehicleNode::onSteeringCmd(const std_msgs::msg::Float64::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received steering command: %f", msg->data);
    QByteArray data = QByteArray::number(msg->data);  // Convert to QByteArray
    sendCommand(data);
}

void VehicleNode::onVelocityCmd(const std_msgs::msg::String::SharedPtr msg)
{
  // TODO: Handle velocity logic here (e.g., forward, backward)
  RCLCPP_INFO(this->get_logger(), "Received velocity command: '%s'", msg->data.c_str());

  // e.g., publish an acknowledgment or control signals to hardware if needed
}

void VehicleNode::onGetPosition(
  const std::shared_ptr<my_vehicle::srv::GetPosition::Request> /* request */,
  std::shared_ptr<my_vehicle::srv::GetPosition::Response> response)
{
  // TODO: Fill in the actual position. For now, just placeholders.
  response->x = 0.0;
  response->y = 0.0;
  response->z = 0.0;

  RCLCPP_INFO(this->get_logger(), "Handling get_position service request");
}

void VehicleNode::onGetBatteryState(
  const std::shared_ptr<my_vehicle::srv::GetBatteryState::Request> /* request */,
  std::shared_ptr<my_vehicle::srv::GetBatteryState::Response> response)
{
  // TODO: Fill in the actual battery state. For now, just placeholders.
  response->battery_percentage = 100.0;

  RCLCPP_INFO(this->get_logger(), "Handling get_battery_state service request");
}

    void VehicleNode::sendCommand(const QByteArray &data) {
        QLocalSocket socket;
        socket.connectToServer("ros2_carclient_channel");

        if (!socket.waitForConnected(3000)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect to carclient");
            return;
        }

        socket.write(data);
        socket.flush();

  //      if (!socket.waitForBytesWritten(3000)) {
  //          RCLCPP_ERROR(this->get_logger(), "Failed to send command");
  //          return;
  //      }

        if (!socket.waitForReadyRead(3000)) {
            RCLCPP_ERROR(this->get_logger(), "No response from carclient");
            return;
        }

        QByteArray response = socket.readAll();
        RCLCPP_INFO(this->get_logger(), "Response from carclient: %s", response.data());

        socket.disconnectFromServer();
    }

}  // namespace my_vehicle

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

