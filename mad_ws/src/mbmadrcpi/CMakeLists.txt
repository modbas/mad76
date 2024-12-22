cmake_minimum_required(VERSION 3.5)
project(mbmadrcpi)

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
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(mbsafe REQUIRED)
find_package(mbsafemsgs REQUIRED)
find_package(mbmad REQUIRED)
find_package(mbmadmsgs REQUIRED)
find_package(Boost COMPONENTS system log REQUIRED)

# check for libwiringPi.so
find_library(WIRINGPI_LIB wiringPi)

# targets
if(WIRINGPI_LIB)
  add_executable(rcnode src/RcNode.cpp)
  target_link_libraries(rcnode ${WIRINGPI_LIB} dl ${Boost_LIBRARIES})
  ament_target_dependencies(rcnode rclcpp std_msgs diagnostic_msgs diagnostic_updater mbsafe mbsafemsgs mbmad mbmadmsgs)
  install(TARGETS
    rcnode
    DESTINATION lib/${PROJECT_NAME}
  )
else()
  message(WARNING "wiringPi library (Raspberry Pi) not found, skipping rcnode")
endif()

ament_package()
