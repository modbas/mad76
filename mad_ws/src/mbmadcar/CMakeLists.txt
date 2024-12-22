cmake_minimum_required(VERSION 3.5)
project(mbmadcar)

option(MAD24 "Enable MAD in 1:24 scale" OFF)

set(CMAKE_VERBOSE_MAKEFILE ON)
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -DBOOST_LOG_DYN_LINK=1)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
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
add_executable(carsimnode src/CarSimNode.cpp)
target_link_libraries(carsimnode dl ${Boost_LIBRARIES})
ament_target_dependencies(carsimnode rclcpp std_msgs diagnostic_msgs diagnostic_updater mbsafe mbsafemsgs mbmad mbmadmsgs)

# add_executable(carsimnode2 src/CarSimNode2.cpp)
# target_link_libraries(carsimnode2 dl ${Boost_LIBRARIES})
# ament_target_dependencies(carsimnode2 rclcpp std_msgs diagnostic_msgs diagnostic_updater mbsafe mbsafemsgs mbmad mbmadmsgs)

add_executable(visionsimnode src/VisionSimNode.cpp)
target_link_libraries(visionsimnode dl ${Boost_LIBRARIES})
ament_target_dependencies(visionsimnode rclcpp std_msgs diagnostic_msgs diagnostic_updater mbsafe mbsafemsgs mbmad mbmadmsgs)

add_executable(carctrlnode src/CarCtrlNode.cpp)
target_link_libraries(carctrlnode dl ${Boost_LIBRARIES})
ament_target_dependencies(carctrlnode rclcpp std_msgs diagnostic_msgs diagnostic_updater mbsafe mbsafemsgs mbmad mbmadmsgs)

# CurbCtrlNode
#add_executable(curbctrlnode src/CrubCtrlNode.cpp)
#target_link_libraries(curbctrlnode dl ${Boost_LIBRARIES})
#ament_target_dependencies(curbctrlnode rclcpp std_msgs diagnostic_msgs diagnostic_updater mbsafe mbsafemsgs mbmad mbmadmsgs)

add_executable(cardisplaynode src/CarDisplayNode.cpp)
ament_target_dependencies(cardisplaynode rclcpp std_msgs visualization_msgs geometry_msgs tf2 tf2_ros tf2_msgs mbmad mbmadmsgs)

# install
install(TARGETS
  carsimnode visionsimnode carctrlnode cardisplaynode # curbctrlnode carsimnode2 
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY
  data
  DESTINATION share/${PROJECT_NAME})


install(DIRECTORY
  scripts/
  DESTINATION lib/${PROJECT_NAME})

ament_package()
