cmake_minimum_required(VERSION 3.8)
project(training)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(turtlesim REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)

# turtle_spawn component
add_library(turtle_spawn SHARED src/TurtleSpawner.cpp)
ament_target_dependencies(turtle_spawn rclcpp rclcpp_components turtlesim)
rclcpp_components_register_node(
  turtle_spawn
  PLUGIN "composition::TurtleSpawner"
  EXECUTABLE turtle_spawn_node
)

# turtle_clear component
add_library(turtle_clear SHARED src/TurtleClearer.cpp)
ament_target_dependencies(turtle_clear rclcpp rclcpp_components turtlesim)
rclcpp_components_register_node(
  turtle_clear
  PLUGIN "composition::TurtleClearer"
  EXECUTABLE turtle_clear_node
)

# circular_motion component
add_library(circular_motion SHARED src/CircularMotion.cpp)
ament_target_dependencies(circular_motion rclcpp rclcpp_components geometry_msgs)
rclcpp_components_register_nodes(
  circular_motion
  "composition::CircularMotion"
)

# Install targets
install(TARGETS
  turtle_spawn
  turtle_clear
  circular_motion
  DESTINATION lib)

ament_package()
