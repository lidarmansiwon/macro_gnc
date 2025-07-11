cmake_minimum_required(VERSION 3.8)
project(ros2_serial_transporter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mk3_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

add_executable(navigation_serial_node src/navigation_serial_node.cpp)
ament_target_dependencies(
  navigation_serial_node
  rclcpp
  mk3_msgs
)

target_include_directories(navigation_serial_node PRIVATE ${Boost_INCLUDE_DIRS})
target_link_libraries(navigation_serial_node ${Boost_LIBRARIES})

add_executable(serial_navigation_receive_node src/serial_navigation_receive_node.cpp)
ament_target_dependencies(
  serial_navigation_receive_node
  rclcpp
  mk3_msgs
)

target_include_directories(serial_navigation_receive_node PRIVATE ${Boost_INCLUDE_DIRS})
target_link_libraries(serial_navigation_receive_node ${Boost_LIBRARIES})


install(
  TARGETS navigation_serial_node
  TARGETS serial_navigation_receive_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
