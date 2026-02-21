cmake_minimum_required(VERSION 3.5)
project(mbmadmsgs)

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
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(mbsafemsgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(include)

# messages
set(msg_files
  "msg/CarInputs.msg"
  "msg/CarLap.msg"
  "msg/CarOutputs.msg"
  "msg/CarOutputsList.msg"
  "msg/CarOutputsExt.msg"
  "msg/CarOutputsExtList.msg"
  "msg/CarObs.msg"
  "msg/CarObsList.msg"
  "msg/CtrlReference.msg"
  "msg/DriveManeuver.msg"
  "srv/TrackGetWaypoints.srv"
  "srv/VisionTransformPoints.srv"
  "srv/ResetLapcounter.srv")

if(MAD24)
  list(APPEND msg_files
  "msg/CarState.msg"
  "msg/DriveRoute.msg"
  "msg/Image.msg"
  "msg/TrackSegment.msg"
  "msg/FaultInjection.msg"
  "srv/TrackGetGraph.srv"
  "srv/TrackGetNearest.srv")
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES builtin_interfaces sensor_msgs mbsafemsgs
)

# install
ament_export_dependencies(rosidl_default_runtime)

ament_package()
