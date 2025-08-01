cmake_minimum_required(VERSION 3.5.1)
project(seatrac_messaging)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(seatrac_driver REQUIRED)

ament_export_dependencies(rosidl_default_runtime)

# Install the Python module directory
install(
  DIRECTORY seatrac_messaging
  DESTINATION lib/${PROJECT_NAME}
)

# Install individual executable scripts
install(
  PROGRAMS
    seatrac_messaging/seatrac_messaging_handler_main.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  PATTERN "*.py"
  PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
)

ament_package()
