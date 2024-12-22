cmake_minimum_required(VERSION 3.5)
project(mbmadlocate)

option(MAD24 "Enable MAD in 1:24 scale" OFF)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -DBOOST_LOG_DYN_LINK=1)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_logging_spdlog REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(mbsafe REQUIRED)
find_package(mbsafemsgs REQUIRED)
find_package(mbmad REQUIRED)
find_package(mbmadmsgs REQUIRED)
find_package(Boost COMPONENTS system log REQUIRED)

#include_directories(include)

if(MAD24)
  add_definitions(-DMAD24)
endif()

# targets
add_executable(locatenode src/LocateNode.cpp)
target_link_libraries(locatenode dl ${Boost_LIBRARIES})
ament_target_dependencies(locatenode rclcpp rcl_logging_spdlog std_msgs diagnostic_msgs diagnostic_updater mbsafe mbsafemsgs mbmad mbmadmsgs)

#ament_export_include_directories(include)

# install
install(TARGETS
  locatenode
  DESTINATION lib/${PROJECT_NAME}
)

#install(DIRECTORY
#  launch
#  DESTINATION share/${PROJECT_NAME})

#install(DIRECTORY
#  data
#  DESTINATION share/${PROJECT_NAME})

#install(
#  DIRECTORY include/
#  DESTINATION include
#)

ament_package()
