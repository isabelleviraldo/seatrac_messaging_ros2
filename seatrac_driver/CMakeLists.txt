cmake_minimum_required(VERSION 3.5)
project(seatrac_driver LANGUAGES C CXX)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# ── Generate custom message interfaces ─────────────────────
rosidl_generate_interfaces(${PROJECT_NAME}
  msg/AcousticFix.msg
  msg/CalibrationActionCommand.msg
  msg/CalibrationActionResponse.msg
  msg/CalibrationGetResponse.msg
  msg/CalibrationSetCommand.msg
  msg/CalibrationSetResponse.msg
  msg/DataErrorStatus.msg
  msg/DataReceiveStatus.msg
  msg/DataSendCommand.msg
  msg/DataSendResponse.msg
  msg/DriverStatus.msg
  msg/Firmware.msg
  msg/Hardware.msg
  msg/NavigationErrorStatus.msg
  msg/NavigationQueryRequestStatus.msg
  msg/NavigationQueryResponseStatus.msg
  msg/NavigationQuerySendCommand.msg
  msg/NavigationQuerySendResponse.msg
  msg/NavigationStatusReceiveStatus.msg
  msg/NavigationStatusSendCommand.msg
  msg/NavigationStatusSendResponse.msg
  msg/PingErrorStatus.msg
  msg/PingRequestStatus.msg
  msg/PingResponseStatus.msg
  msg/PingSendCommand.msg
  msg/PingSendResponse.msg
  msg/SerialInternal.msg
  msg/SettingsGetResponse.msg
  msg/SettingsLoadResponse.msg
  msg/SettingsResetResponse.msg
  msg/SettingsSaveResponse.msg
  msg/SettingsSetCommand.msg
  msg/SettingsSetResponse.msg
  msg/StatusCommand.msg
  msg/StatusConfigGetResponse.msg
  msg/StatusConfigSetCommand.msg
  msg/StatusConfigSetResponse.msg
  msg/StatusResponse.msg
  msg/SystemAliveResponse.msg
  msg/SystemInfoResponse.msg
  msg/SystemRebootResponse.msg
  msg/TransceiverAnalyzeResponse.msg
  msg/TransceiverFixStatus.msg
  msg/TransceiverStatusResponse.msg

  DEPENDENCIES
    std_msgs
    geometry_msgs
    nav_msgs
    builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

# Install the Python module directory
install(
  DIRECTORY seatrac_driver
  DESTINATION lib/${PROJECT_NAME}
)

# Install individual executable scripts if needed
install(
  PROGRAMS
    seatrac_driver/seatrac_driver_node.py
    seatrac_driver/seatrac_driver_stub_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  PATTERN "*.py"
)

ament_package()
