cmake_minimum_required(VERSION 3.8)
project(my_cpp_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include_directories("/opt/ros/jazzy/include")
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(my_robot_interfaces REQUIRED)

add_executable(cpp_node src/my_first_node.cpp)
add_executable(robot_news_station src/robot_news_station.cpp)
add_executable(smartphone src/smartphone.cpp)
add_executable(number_publisher src/number_publisher.cpp)
add_executable(number_counter src/number_counter.cpp)
add_executable(add_two_ints_server src/add_two_ints_server.cpp)


ament_target_dependencies( robot_news_station
  rclcpp
  example_interfaces
)

ament_target_dependencies( cpp_node
  rclcpp
  example_interfaces
)

ament_target_dependencies( smartphone
  rclcpp
  example_interfaces
)

ament_target_dependencies( number_publisher
  rclcpp
  example_interfaces
)

ament_target_dependencies( number_counter
  rclcpp
  example_interfaces
)

ament_target_dependencies( add_two_ints_server
  rclcpp
  example_interfaces
)


add_executable(add_two_ints_client_no_oop src/add_two_ints_client_no_oop.cpp)
ament_target_dependencies( add_two_ints_client_no_oop
  rclcpp
  example_interfaces
)

add_executable(add_two_ints_client src/add_two_ints_client.cpp)
ament_target_dependencies( add_two_ints_client
  rclcpp
  example_interfaces
)

add_executable(hardware_status_publisher src/hardware_status_publisher.cpp)
ament_target_dependencies( hardware_status_publisher
  rclcpp
  my_robot_interfaces
)

target_include_directories(cpp_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install( TARGETS
  cpp_node
  robot_news_station
  smartphone
  number_publisher
  number_counter
  add_two_ints_server
  add_two_ints_client_no_oop
  add_two_ints_client
  hardware_status_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
