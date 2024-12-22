cmake_minimum_required(VERSION 3.5)
project(mbmad)

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

include_directories(include)

# targets
set(MODULES src/CarParameters.cpp src/Spline.cpp)

if(MAD24)
  add_definitions(-DMAD24)
  list(APPEND MODULES src/CircleSegment.cpp src/ClothoidSegment.cpp src/Random.cpp src/StraightSegment.cpp src/Track.cpp src/TrackCircuit.cpp src/TrackSegment.cpp)
endif()

add_library(${PROJECT_NAME} SHARED ${MODULES})

#target_link_libraries(${PROJECT_NAME} lttng-ust)
ament_target_dependencies(${PROJECT_NAME} rclcpp)

# install
ament_export_libraries(${PROJECT_NAME})
ament_export_include_directories(include)

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  include/
  DESTINATION include
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
