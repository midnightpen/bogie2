cmake_minimum_required(VERSION 3.8)
project(my_cpp_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(cpp_node src/my_first_node.cpp)
ament_target_dependencies(cpp_node rclcpp)

add_executable(cpp_oop_node src/my_first_oop_node.cpp)
ament_target_dependencies(cpp_oop_node rclcpp)

add_executable(cpp_pub_node src/cpp_simple_pub.cpp)
ament_target_dependencies(cpp_pub_node rclcpp std_msgs)

add_executable(cpp_sub_node src/cpp_simple_sub.cpp)
ament_target_dependencies(cpp_sub_node rclcpp std_msgs)

install(TARGETS
  cpp_node
  cpp_oop_node
  cpp_pub_node
  cpp_sub_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
