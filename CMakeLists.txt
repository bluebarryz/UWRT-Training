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
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
	"msg/TurtleDist.msg"
	DEPENDENCIES builtin_interfaces
	)
ament_export_dependencies(rosidl_default_runtime)

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

# dist_publisher component
add_library(dist_publisher SHARED
  src/DistancePublisher.cpp
)
ament_target_dependencies(dist_publisher
  rclcpp
  rclcpp_components
  turtlesim
)
rosidl_target_interfaces(dist_publisher ${PROJECT_NAME} "rosidl_typesupport_cpp")
rclcpp_components_register_nodes(dist_publisher "composition::DistancePublisher")



# Install targets
install(TARGETS
  turtle_spawn
  turtle_clear
  circular_motion
  dist_publisher
  ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib 
	RUNTIME DESTINATION bin)


ament_package()
