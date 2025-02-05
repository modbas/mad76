cmake_minimum_required(VERSION 3.8)
project(mbmadvisionaruco)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(PkgConfig)
find_package(OpenCV REQUIRED)
find_package(Boost COMPONENTS system log REQUIRED)
find_package(mbmad REQUIRED)
find_package(mbmadmsgs REQUIRED)
find_package(mbsafe REQUIRED)
find_package(mbsafemsgs REQUIRED)

add_executable(visionnode src/VisionNode.cpp src/Frame.hpp)
target_include_directories(visionnode PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  )
target_compile_features(visionnode PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(visionnode rclcpp std_msgs diagnostic_msgs diagnostic_updater sensor_msgs image_transport cv_bridge mbmad mbmadmsgs mbsafe mbsafemsgs)
target_link_libraries(visionnode dl ${Boost_LIBRARIES} ${OpenCV_LIBRARIES})
install(TARGETS visionnode
  DESTINATION lib/${PROJECT_NAME})

add_executable(create_board src/create_board.cpp)
target_include_directories(create_board PUBLIC
  ${OpenCV_INCLUDE_DIRS}
  )
target_link_libraries(create_board ${OpenCV_LIBRARIES})
install(TARGETS create_board
    DESTINATION lib/${PROJECT_NAME})

add_executable(create_board_charuco src/create_board_charuco.cpp)
target_include_directories(create_board_charuco PUBLIC
  ${OpenCV_INCLUDE_DIRS}
  )
target_link_libraries(create_board_charuco ${OpenCV_LIBRARIES})
install(TARGETS create_board_charuco
    DESTINATION lib/${PROJECT_NAME})

# add_executable(delaytestnode src/DelayTestNode.cpp)
# target_include_directories(delaytestnode PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
#   ${OpenCV_INCLUDE_DIRS}
#   ${rclcpp_INCLUDE_DIRS}
#   )
# target_compile_features(delaytestnode PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
# ament_target_dependencies(delaytestnode rclcpp std_msgs diagnostic_msgs diagnostic_updater sensor_msgs image_transport cv_bridge mbmad mbmadmsgs mbsafemsgs)
# target_link_libraries(delaytestnode wiringPi dl ${Boost_LIBRARIES} ${OpenCV_LIBRARIES})
# install(TARGETS delaytestnode
#   DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY
  scripts/
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
   data
   DESTINATION share/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
