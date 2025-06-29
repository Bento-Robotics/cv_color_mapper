cmake_minimum_required(VERSION 3.8)
project(cv_color_mapper)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(color_map_node src/color_map_node.cpp)

ament_target_dependencies(color_map_node rclcpp sensor_msgs cv_bridge OpenCV image_transport)

target_link_libraries(color_map_node
  ${OpenCV_LIBS}
)

install(
  DIRECTORY
    launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  color_map_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
