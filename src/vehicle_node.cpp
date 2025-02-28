#include "my_vehicle/vehicle_node.hpp"
#include <QLocalSocket>
#include <QLocalServer>


namespace my_vehicle
{

VehicleNode::VehicleNode()
: Node("vehicle_node")
{
        RCLCPP_INFO(this->get_logger(), "Creating vehicle");
        qDebug() << "Creating vehicle";

     // Create the service server
    set_vehicle_id_service_ = create_service<my_vehicle::srv::SetVehicleId>(
        "set_vehicle_id",
        std::bind(&VehicleNode::setVehicleIdCallback, this, std::placeholders::_1, std::placeholders::_2));

  // Create subscriptions          
      directcontrol_sub_ = create_subscription<my_vehicle::msg::DirectControl>(
        "directcontrol_cmd", 10,
        std::bind(&VehicleNode::onDirectControlCmd, this, std::placeholders::_1));

    steering_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "steering_cmd", 10,
        std::bind(&VehicleNode::onSteeringCmd, this, std::placeholders::_1));

    velocity_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "velocity_cmd", 10,
        std::bind(&VehicleNode::onVelocityCmd, this, std::placeholders::_1));

    hydraulics_sub_ = this->create_subscription<my_vehicle::msg::Hydraulics>(
        "hydraulics_cmd", 10,
        std::bind(&VehicleNode::onHydraulicsCmd, this, std::placeholders::_1));

        
  terminal_sub_ = create_subscription<std_msgs::msg::String>(
    "terminal_cmd",
    10,
    std::bind(&VehicleNode::onTerminalCmd, this, std::placeholders::_1)
  );

        RCLCPP_INFO(this->get_logger(), "Creating vehicle (1)");

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
  
        RCLCPP_INFO(this->get_logger(), "Creating vehicle (2)");

  
      // Initialize the publisher for terminal output
    terminal_publisher_ = this->create_publisher<std_msgs::msg::String>("terminal_output", 10);

    // Set up the terminal server
    setupTerminalServer();
  
      // Process connections in a loop (spun by the ROS event loop)
    this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&VehicleNode::processTerminalConnections, this));
  

}


VehicleNode::~VehicleNode() {
    if (terminal_server_) {
        terminal_server_->close();
        delete terminal_server_;
    }
}

QByteArray VehicleNode::standardMessageStart(int id)
{
    QByteArray data;
/*    data.append(static_cast<char>(2));  // Append the initial 0
    data.append(static_cast<char>(6));  // Append the initial 0
    data.append(static_cast<char>(0));  // Append the initial 0
*/


 //   data.append(static_cast<char>(vehicle_id_));
    data.append(static_cast<char>(id));  // Append the command identifier
    return data;
}

void VehicleNode::onSteeringCmd(const std_msgs::msg::Float64::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received steering command: %f", msg->data);

    QByteArray data=standardMessageStart(121);
   
    // Append the Float64 value in binary format
    double steering_value = msg->data;
    QByteArray value_bytes(reinterpret_cast<const char*>(&steering_value), sizeof(double));
    data.append(value_bytes);

    RCLCPP_INFO(this->get_logger(), "Sending QByteArray: [%s]", data.toHex(':').toStdString().c_str());

    // Send the constructed QByteArray
    sendCommand(data);
}

void VehicleNode::onVelocityCmd(const std_msgs::msg::Float64::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received velocity command: %f", msg->data);

    // Create a QByteArray with the first byte set to 121
    QByteArray data;
    data.append(static_cast<char>(0));  // Append the initial 0
 //   data.append(static_cast<char>(vehicle_id_));
    data.append(static_cast<char>(121));  // Append the steering command identifier

   
    // Append the Float64 value in binary format
    double velocity_value = msg->data;
    QByteArray value_bytes(reinterpret_cast<const char*>(&velocity_value), sizeof(double));
    data.append(value_bytes);

    RCLCPP_INFO(this->get_logger(), "Sending QByteArray: [%s]", data.toHex(':').toStdString().c_str());

    // Send the constructed QByteArray
    sendCommand(data);
}

void VehicleNode::onHydraulicsCmd(const my_vehicle::msg::Hydraulics::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received hydraulics command - pos: %d, move: %d", 
                msg->pos, msg->move);

    QByteArray data=standardMessageStart(88);

    data.append(static_cast<char>(msg->pos));
    data.append(static_cast<char>(msg->move));


    RCLCPP_INFO(this->get_logger(), "Sending combined command");
    sendCommand(data);
}


void VehicleNode::onTerminalCmd(const std_msgs::msg::String::SharedPtr msg)
{
  // TODO: Handle velocity logic here (e.g., forward, backward)
  RCLCPP_INFO(this->get_logger(), "Received terminal command: '%s'", msg->data.c_str());

    QByteArray data=standardMessageStart(1);
    data.append(msg->data.c_str());

    RCLCPP_INFO(this->get_logger(), "Sending terminal command");
    sendCommand(data);
    	
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
    
    
    void VehicleNode::onDirectControlCmd(const my_vehicle::msg::DirectControl::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received command - Throttle: %f, Steering: %f", 
                msg->throttle, msg->steering);

    QByteArray data;

    data.append(static_cast<char>(0));  // Append the initial 0
    // Add the vehicle ID (byte 1)
    data.append(static_cast<char>(vehicle_id_));
    // Add the command identifier (byte 2, e.g., 121)
    data.append(static_cast<char>(121));

    // Add the velocity value (bytes 11–18)
    QByteArray throttle_bytes(reinterpret_cast<const char*>(&msg->throttle), sizeof(double));
    data.append(throttle_bytes);

    // Add the steering value (bytes 3–10)
    QByteArray steering_bytes(reinterpret_cast<const char*>(&msg->steering), sizeof(double));
    data.append(steering_bytes);

    RCLCPP_INFO(this->get_logger(), "Sending combined command");
    sendCommand(data);
}



void VehicleNode::setVehicleIdCallback(
    const std::shared_ptr<my_vehicle::srv::SetVehicleId::Request> request,
    std::shared_ptr<my_vehicle::srv::SetVehicleId::Response> response) {
    RCLCPP_INFO(this->get_logger(), "Service called with id: %d", request->id);

    vehicle_id_ = request->id;
    response->success = true;
    response->message = "Vehicle ID updated successfully";
}

void VehicleNode::setupTerminalServer() {
    // Initialize the local server for terminal messages
    
      RCLCPP_INFO(this->get_logger(), "Setting up terminal server");

    terminal_server_ = new QLocalServer();
    if (!terminal_server_->listen("ros2_carclient_terminal_channel")) {
        RCLCPP_ERROR(this->get_logger(), "Failed to start terminal server: %s",
                     terminal_server_->errorString().toStdString().c_str());
    } else {
        RCLCPP_INFO(this->get_logger(), "Listening on 'ros2_carclient_terminal_channel' for terminal messages.");
    }
}

void VehicleNode::processTerminalConnections() {
    if (!terminal_server_) return;

    while (QLocalSocket *client_connection = terminal_server_->nextPendingConnection()) {
        processClientSocket(client_connection);
    }
}

void VehicleNode::processClientSocket(QLocalSocket *client_connection) {
    if (!client_connection) return;

    while (client_connection->bytesAvailable()) {
        QByteArray data = client_connection->readAll();
        if (!data.isEmpty()) {
            QString message = QString::fromUtf8(data);

            // Publish the message to the ROS 2 topic
            auto msg = std_msgs::msg::String();
            msg.data = message.toStdString();
            terminal_publisher_->publish(msg);

            RCLCPP_INFO(this->get_logger(), "Published terminal message: %s", message.toStdString().c_str());
        }
    }

    client_connection->close();
    delete client_connection;
}

}  // namespace my_vehicle


