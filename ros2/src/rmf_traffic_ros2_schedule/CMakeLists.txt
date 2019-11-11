cmake_minimum_required(VERSION 3.5)
project(rmf_traffic_ros2_schedule)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(GNUInstallDirs)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmf_traffic REQUIRED)
find_package(rmf_traffic_msgs REQUIRED)

include(GNUInstallDirs)

add_library(rmf_traffic_ros2_schedule SHARED src/ScheduleNode.cpp)

target_link_libraries(rmf_traffic_ros2_schedule
  PUBLIC
    rmf_traffic::rmf_traffic
    ${rmf_traffic_msgs_LIBRARIES}
    ${rclcpp_LIBRARIES}
)

target_include_directories(rmf_traffic_ros2_schedule
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${rmf_traffic_msgs_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
)

install(
  DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(
  TARGETS rmf_traffic_ros2_schedule
  EXPORT  rmf_traffic_ros2_schedule
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

ament_package()
