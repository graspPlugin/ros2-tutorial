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

# Find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Generate custom interfaces
set(msg_files
  "msg/SampleMsg.msg"
)
set(srv_files
  "srv/SampleSrv.srv"
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
)
ament_export_dependencies(rosidl_default_runtime)

# hello world by c++ std library
add_executable(hello_world src/hello_world.cpp)

# simple node
add_executable(simple_node src/simple_node.cpp)
ament_target_dependencies(simple_node rclcpp)

# timer
add_executable(timer src/timer.cpp)
ament_target_dependencies(timer rclcpp)

# publisher
add_executable(publisher src/publisher.cpp)
ament_target_dependencies(publisher rclcpp std_msgs)

# subscriber
add_executable(subscriber src/subscriber.cpp)
ament_target_dependencies(subscriber rclcpp std_msgs)

# sample msg publisher
add_executable(sample_msg_publisher src/sample_msg_publisher.cpp)
ament_target_dependencies(sample_msg_publisher rclcpp)
rosidl_target_interfaces(sample_msg_publisher ${PROJECT_NAME} "rosidl_typesupport_cpp")

# server
add_executable(server src/server.cpp)
ament_target_dependencies(server rclcpp)
rosidl_target_interfaces(server ${PROJECT_NAME} "rosidl_typesupport_cpp")

# client
add_executable(client src/client.cpp)
ament_target_dependencies(client rclcpp)
rosidl_target_interfaces(client ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
  hello_world
  simple_node
  timer
  publisher
  subscriber
  sample_msg_publisher
  server
  client
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()