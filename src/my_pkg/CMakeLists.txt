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
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# custom interface
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
ament_auto_add_executable(hello_world src/hello_world.cpp)

# simple node
ament_auto_add_executable(simple_node src/simple_node.cpp)

# timer
ament_auto_add_executable(timer src/timer.cpp)

# publisher
ament_auto_add_executable(publisher src/publisher.cpp)

# subscriber
ament_auto_add_executable(subscriber src/subscriber.cpp)

# sample msg publisher
# add_executable(sample_msg_publisher src/sample_msg_publisher.cpp)
# ament_target_dependencies(sample_msg_publisher rclcpp std_msgs)

ament_auto_package()