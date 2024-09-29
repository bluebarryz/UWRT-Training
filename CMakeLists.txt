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

# turtle_spawn component
add_library(turtle_spawn SHARED src/TurtleSpawner.cpp)
ament_target_dependencies(turtle_spawn rclcpp rclcpp_components turtlesim)
rclcpp_components_register_node(turtle_spawn
  PLUGIN "composition::TurtleSpawner"
  EXECUTABLE turtle_spawn_node
)

# Create an executable
add_executable(turtle_spawn_executable src/main.cpp)

# Link the turtle_spawn library with the executable
target_link_libraries(turtle_spawn_executable
  turtle_spawn
)

# Specify dependencies for the executable
ament_target_dependencies(turtle_spawn_executable
  rclcpp
  rclcpp_components
)

# Install targets
install(TARGETS
  turtle_spawn_executable
  turtle_spawn
  DESTINATION lib/${PROJECT_NAME})

ament_package()
