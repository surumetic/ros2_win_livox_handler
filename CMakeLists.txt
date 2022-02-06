cmake_minimum_required(VERSION 3.8)
project(ros2_win_livox_handler)


# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

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
find_package(tf2_geometry_msgs)
# find_package(pcl_conversions)

SET( LIBS livox_sdk_static )

include_directories(
        sdk_core/include
        sdk_core/third_party/FastCRC
        sdk_core/third_party/spdlog
        sdk_core/third_party/cmdline
        include/ros2_win_livox_handler
)

link_directories (
  ./lib
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(ros2_win_livox_handler src/livox_handler.cpp src/callback.cpp)
target_link_libraries(ros2_win_livox_handler ${LIBS} )
ament_target_dependencies(ros2_win_livox_handler 
  rclcpp 
  std_msgs 
  sensor_msgs
  tf2_geometry_msgs
  
  # pcl_conversions
)

install(TARGETS
  ros2_win_livox_handler
  DESTINATION lib/${PROJECT_NAME})

ament_package()
