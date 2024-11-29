cmake_minimum_required(VERSION 3.5)
project(john_controls)

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
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(Eigen3 REQUIRED)
# find_package(john_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2 REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(thrust_allocator 
  src/thrust_allocator.cpp
)
target_include_directories(thrust_allocator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS thrust_allocator
  DESTINATION lib/${PROJECT_NAME}
)
add_library(john_controls SHARED
  src/thrust_allocator.cpp
)

rclcpp_components_register_nodes(john_controls "john_controls::ThrustAllocator")

target_compile_definitions(john_controls
  PRIVATE "john_controls_CPP_BUILDING_DLL"
)

ament_target_dependencies(john_controls
  "rclcpp"
  "rclcpp_components"
  "Eigen3"
 # "john_interfaces"
  "geometry_msgs"
  "sensor_msgs"
  "OpenCV"
  "tf2"
  "nav_msgs"
)

install(TARGETS
  john_controls
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

 
install(DIRECTORY
  test/config
  DESTINATION share/${PROJECT_NAME}/test
)

# Install Python modules
# ament_python_install_package(${PROJECT_NAME})

# Install Python executables
#install(PROGRAMS
#  john_controls/auv_transform_publisher.py
#  john_controls/waypoint_marker_tester.py
#  john_controls/predetermined_route.py
#  john_controls/predetermined_thrust.py
#  DESTINATION lib/${PROJECT_NAME}
#)

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(
    test/test_thrust_allocator_launch.py
  )
endif()

ament_package()