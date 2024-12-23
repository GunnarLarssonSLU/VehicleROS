cmake_minimum_required(VERSION 3.8)
project(my_vehicle)

# Default to C++14 (or C++17 if you prefer) 
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# If you are using custom .srv files:
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetPosition.srv"
  "srv/GetBatteryState.srv"
)

# Standard find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Network)  # Add Qt5 modules

# If you want to use the generated service definitions in C++:
ament_export_dependencies(rosidl_default_runtime)

include_directories(
  include ${Qt5Core_INCLUDE_DIRS} ${Qt5Network_INCLUDE_DIRS}
)

add_definitions(${Qt5_DEFINITIONS})


# Generate the library/executable
add_executable(vehicle_node src/vehicle_node.cpp)
ament_target_dependencies(vehicle_node rclcpp std_msgs rosidl_default_runtime)
target_link_libraries(vehicle_node Qt5::Core Qt5::Network)

install(TARGETS
  vehicle_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

# This is required so that ROS 2 can locate your interfaces (if you have them)
rosidl_get_typesupport_target(typesupport_target_name ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(vehicle_node ${typesupport_target_name})

ament_package()

