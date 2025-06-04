cmake_minimum_required(VERSION 3.8)
project(navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(INCLUDE_DIRECTORIES 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};/usr/share/cmake/geographiclib")


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)       
find_package(std_msgs REQUIRED)    
find_package(mk3_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(GeographicLib REQUIRED)

include_directories(include)

# Include install path
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

# Add your test executable
add_executable(lpf_sample src/tool/lpf_sample.cpp src/tool/low_pass_filter.cpp)
add_executable(quaternion_sample src/tool/quaternion_sample.cpp src/tool/quaternion_utils.cpp)
add_executable(odom_navigation src/use_slam/main.cpp src/use_slam/odom_navigation.cpp src/tool/quaternion_utils.cpp)
ament_target_dependencies(odom_navigation rclcpp nav_msgs mk3_msgs sensor_msgs geometry_msgs)

add_executable(gps_navigation src/use_gps/main.cpp src/use_gps/gps_navigation.cpp src/tool/quaternion_utils.cpp)
ament_target_dependencies(gps_navigation rclcpp nav_msgs mk3_msgs sensor_msgs geometry_msgs GeographicLib)

# Set include directories
target_include_directories(gps_navigation PUBLIC ${INCLUDE_DIRECTORIES})
target_include_directories(odom_navigation PUBLIC ${INCLUDE_DIRECTORIES})
target_include_directories(lpf_sample PUBLIC ${INCLUDE_DIRECTORIES})
target_include_directories(quaternion_sample PUBLIC ${INCLUDE_DIRECTORIES})

install(TARGETS
  lpf_sample
  quaternion_sample
  odom_navigation
  gps_navigation

  DESTINATION lib/${PROJECT_NAME})

target_link_libraries(gps_navigation
  ${GeographicLib_LIBRARIES} 
)

# Linting and testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
