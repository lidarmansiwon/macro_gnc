cmake_minimum_required(VERSION 3.8)
project(control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)       
find_package(std_msgs REQUIRED)    
find_package(mk3_msgs REQUIRED)

include_directories(include)
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY param/
  DESTINATION share/${PROJECT_NAME}/param
)

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME})

add_executable(pid_control src/pid/main.cpp src/pid/pid_control.cpp)
ament_target_dependencies(pid_control rclcpp mk3_msgs)

target_include_directories(pid_control PUBLIC ${INCLUDE_DIRECTORIES})

install(TARGETS
  pid_control

  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
