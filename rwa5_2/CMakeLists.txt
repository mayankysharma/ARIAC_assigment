cmake_minimum_required(VERSION 3.8)
project(rwa5_2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ariac_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_srvs REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(moveit_msgs REQUIRED)

find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(robot_commander_msgs REQUIRED)

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

set(
  ARIAC_INCLUDE_DIRS 
  rclcpp 
  std_msgs
  ariac_msgs
  tf2_ros
  geometry_msgs
  tf2
  orocos_kdl
  tf2_ros
  tf2_geometry_msgs
  moveit_msgs
  geometric_shapes
  moveit_ros_planning_interface
  
)

install(DIRECTORY
  launch
  doc
  meshes
  config
  DESTINATION share/${PROJECT_NAME}/
)


install(PROGRAMS
  rwa5_2/comp_state.py
  rwa5_2/ariac_interface_util.py
  rwa5_2/ariac_interface_main.py
  rwa5_2/process_order.py
  rwa5_2/custom_timer.py
  rwa5_2/read_store_orders.py
  rwa5_2/ship_orders.py
  rwa5_2/submit_orders.py
  rwa5_2/utils.py
  rwa5_2/sensor_read.py
  rwa5_2/robot_move.py
  DESTINATION lib/${PROJECT_NAME})

include_directories(${ariac_msgs_INCLUDE_DIRS})
# # Add include directory for header files
include_directories(include)
add_executable(robot_ariac src/robot_ariac.cpp)

# Link dependencies to the executable
ament_target_dependencies(robot_ariac
${ARIAC_INCLUDE_DIRS}
robot_commander_msgs
)

install(TARGETS
robot_ariac
DESTINATION lib/${PROJECT_NAME}
)
ament_package()
