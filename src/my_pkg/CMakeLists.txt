cmake_minimum_required(VERSION 3.5)
project(my_pkg)

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
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# custom message
set(msg_files
  "msg/Str.msg"
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)
ament_export_dependencies(rosidl_default_runtime)

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

ament_package()

# hello world by c++ std library
add_executable(hello_world src/hello_world.cpp)

# hello world by ros node
add_executable(hello_ros_world src/hello_ros_world.cpp)
ament_target_dependencies(hello_ros_world rclcpp)

# simple node
add_executable(simple_node src/simple_node.cpp)
ament_target_dependencies(simple_node rclcpp)

# class test
add_executable(class_test src/class_test.cpp)

# timer
add_executable(timer src/timer.cpp)
ament_target_dependencies(timer rclcpp)

# publisher
add_executable(publisher src/publisher.cpp)
ament_target_dependencies(publisher rclcpp)
rosidl_target_interfaces(publisher ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(
  TARGETS
  hello_world
  hello_ros_world
  class_test
  simple_node
  timer
  publisher
  DESTINATION lib/${PROJECT_NAME}
)