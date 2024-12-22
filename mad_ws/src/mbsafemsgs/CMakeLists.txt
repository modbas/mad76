cmake_minimum_required(VERSION 3.5)
project(mbsafemsgs)

option(MAD24 "Enable MAD in 1:24 scale" OFF)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(include)

# messages
set(msg_files
  "msg/Checkpoint.msg"
  "msg/CheckpointSequence.msg")

if(MAD24)
  list(APPEND msg_files
  "msg/Token.msg"
  "msg/FaultInjection.msg")
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES builtin_interfaces
)

# install
ament_export_dependencies(rosidl_default_runtime)

ament_package()
