cmake_minimum_required(VERSION 3.5)
project(mbsafe)

option(MAD24 "Enable MAD in 1:24 scale" OFF)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -DBOOST_LOG_DYN_LINK=1)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(mbsafemsgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(Boost COMPONENTS system log REQUIRED)

include_directories(include)

# targets
set(MODULES src/Platform.cpp src/mbsafe-tp.cpp)

if(MAD24)
  add_definitions(-DMAD24)
  list(APPEND MODULES src/GtestFile.cpp)
endif()

add_library(${PROJECT_NAME} SHARED ${MODULES})
target_link_libraries(${PROJECT_NAME} lttng-ust ${Boost_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME} rclcpp mbsafemsgs)

# ament_add_gtest(safetest test/safetest.cpp test/SafeTestFixture.cpp)
# target_link_libraries(safetest dl ${PROJECT_NAME} ${Boost_LIBRARIES})
# ament_target_dependencies(safetest rclcpp std_msgs diagnostic_msgs mbmad mbmadmsgs mbsafe mbsafemsgs)

# install
ament_export_libraries(${PROJECT_NAME})
ament_export_include_directories(include)

install(TARGETS
  ${PROJECT_NAME}
  # safetest
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  include/
  DESTINATION include
)

if(MAD24)
  install(DIRECTORY
    scripts/
    DESTINATION lib/${PROJECT_NAME})
endif()

ament_package()
