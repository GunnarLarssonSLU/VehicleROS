cmake_minimum_required(VERSION 3.8)
project(my_vehicle)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)  # ✅ Ensure type support is found
find_package(Qt5 REQUIRED COMPONENTS Core Network)

# ✅ Generate interfaces for services and messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetPosition.srv"
  "srv/GetBatteryState.srv"
  "srv/SetVehicleId.srv"
  "msg/DirectControl.msg"
  "msg/Hydraulics.msg"
  DEPENDENCIES std_msgs
)

# ✅ Ensure generated headers are included
include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
  ${CMAKE_INSTALL_PREFIX}/include
)

# ✅ Define the library for the `VehicleNode` class
add_library(my_vehicle_library
  src/vehicle_node.cpp
)

set_target_properties(my_vehicle_library PROPERTIES EXPORT_NAME my_vehicle_library)

# ✅ Link ROS 2 and Qt libraries
ament_target_dependencies(my_vehicle_library
  rclcpp
  std_msgs
  rosidl_default_runtime
  rosidl_typesupport_cpp  # ✅ Ensure type support library is linked
)

target_link_libraries(my_vehicle_library
  Qt5::Core
  Qt5::Network
)

# ✅ Define the executable for `my_vehicle_node`
add_executable(my_vehicle_node src/main.cpp)
target_link_libraries(my_vehicle_node my_vehicle_library)

# ✅ Ensure that the type support is linked to `my_vehicle_node`
rosidl_get_typesupport_target(typesupport_target_name ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(my_vehicle_node ${typesupport_target_name})

# ✅ Link generated service/message interfaces (modern method)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(my_vehicle_library ${cpp_typesupport_target})



# ✅ Install the library and executable
install(TARGETS
  my_vehicle_library
  my_vehicle_node
  EXPORT my_vehicle_library
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)

install(EXPORT
  my_vehicle_library
  NAMESPACE my_vehicle::
  DESTINATION lib/cmake/my_vehicle)

install(DIRECTORY include/my_vehicle
  DESTINATION include
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)


install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp/
  DESTINATION include
)

# ✅ Ensure ONLY `my_vehicle_library` is exported
ament_export_dependencies(rclcpp std_msgs rosidl_default_runtime rosidl_typesupport_cpp Qt5)
ament_export_targets(my_vehicle_library HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(my_vehicle_library)

ament_package()

