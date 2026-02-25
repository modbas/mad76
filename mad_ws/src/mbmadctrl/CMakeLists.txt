cmake_minimum_required(VERSION 3.5)
project(mbmadctrl)

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

#include_directories(include)
# FT
if(CMAKE_SYSTEM_PROCESSOR MATCHES "arm" OR CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    set(SRCDIR src_aarch64)
else()
    set(SRCDIR src)
endif()

# targets
add_executable(carctrlnode ${SRCDIR}/CarCtrlNode.cpp ${SRCDIR}/Motion.c ${SRCDIR}/rt_nonfinite.c ${SRCDIR}/rtGetNaN.c)
target_link_libraries(carctrlnode dl ${Boost_LIBRARIES})
ament_target_dependencies(carctrlnode rclcpp std_msgs diagnostic_msgs diagnostic_updater mbsafe mbsafemsgs mbmad mbmadmsgs)

#ament_export_include_directories(include)

# install
install(TARGETS
  carctrlnode
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

#install(DIRECTORY
#  data
#  DESTINATION share/${PROJECT_NAME})

#install(
#  DIRECTORY include/
#  DESTINATION include
#)

ament_package()
