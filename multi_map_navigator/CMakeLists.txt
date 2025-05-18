cmake_minimum_required(VERSION 3.8)
project(multi_map_navigator)

# Compile options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(multi_map_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(SQLite3 REQUIRED)



# Node executable
add_executable(multi_map_navigator_node src/multi_map.cpp)
target_link_libraries(multi_map_navigator_node SQLite::SQLite3)
ament_target_dependencies(multi_map_navigator_node
  rclcpp
  rclcpp_action
  std_msgs
  geometry_msgs
  multi_map_action
  nav2_msgs
)

# Install
install(TARGETS
  multi_map_navigator_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
