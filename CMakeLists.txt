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
find_package(rclcpp_action REQUIRED)



rosidl_generate_interfaces(${PROJECT_NAME}
	"msg/TurtleDist.msg"
  "action/Move.action"
  "srv/Reset.srv"
	DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
	)
ament_export_dependencies(rosidl_default_runtime)

# turtle_spawn component
add_library(turtle_spawn SHARED src/TurtleSpawner.cpp src/Constants.cpp)
target_compile_definitions(turtle_spawn
	PRIVATE "COMPOSITION_BUILDING_DLL"
)
ament_target_dependencies(turtle_spawn rclcpp rclcpp_components turtlesim)
rosidl_target_interfaces(turtle_spawn ${PROJECT_NAME} "rosidl_typesupport_cpp") # need for custom srv
rclcpp_components_register_node(
  turtle_spawn
  PLUGIN "composition::TurtleSpawner"
  EXECUTABLE turtle_spawn_node
)

# turtle_clear component
add_library(turtle_clear SHARED src/TurtleClearer.cpp)
target_compile_definitions(turtle_clear
	PRIVATE "COMPOSITION_BUILDING_DLL"
)
ament_target_dependencies(turtle_clear rclcpp rclcpp_components turtlesim)
rosidl_target_interfaces(turtle_clear ${PROJECT_NAME} "rosidl_typesupport_cpp") # need for custom srv
rclcpp_components_register_nodes(turtle_clear "composition::TurtleClearer")

# circular_motion component
add_library(circular_motion SHARED src/CircularMotion.cpp)
target_compile_definitions(circular_motion
	PRIVATE "COMPOSITION_BUILDING_DLL"
)
ament_target_dependencies(circular_motion rclcpp rclcpp_components geometry_msgs)
rosidl_target_interfaces(circular_motion ${PROJECT_NAME} "rosidl_typesupport_cpp") # need for custom srv
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


# turtle_move component
add_library(turtle_move SHARED
  src/TurtleMover.cpp
)
target_compile_definitions(turtle_move
	PRIVATE "SOFTWARE_TRAINING_DLL")
ament_target_dependencies(turtle_move
       "rclcpp"
       "rclcpp_components" 
       "turtlesim"
       "rclcpp_action"
       "std_msgs"
       "geometry_msgs")
rosidl_target_interfaces(turtle_move ${PROJECT_NAME} "rosidl_typesupport_cpp") # needed for custom action
# with this we can exexute the component as a node - ros2 run training moving_action_server
rclcpp_components_register_node(turtle_move PLUGIN "composition::TurtleMover" EXECUTABLE moving_action_server)

# turtle_reset component
add_library(turtle_reset SHARED src/TurtleReset.cpp src/Constants.cpp)
target_compile_definitions(turtle_reset
	PRIVATE "COMPOSITION_BUILDING_DLL"
)
ament_target_dependencies(turtle_reset
	"rclcpp"
	"rclcpp_components"
	"turtlesim"
	"std_msgs")
rosidl_target_interfaces(turtle_reset ${PROJECT_NAME} "rosidl_typesupport_cpp") # need for custom srv
rclcpp_components_register_nodes(turtle_reset "composition::TurtleReset")


# Install targets
install(TARGETS
  turtle_spawn
  turtle_clear
  circular_motion
  dist_publisher
  turtle_move
  turtle_reset
  ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib 
	RUNTIME DESTINATION bin)


ament_package()
