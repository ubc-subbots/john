cmake_minimum_required(VERSION 3.5)
project(john_controls)

# Include directories
include_directories(include)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# Add the shared library for the thrust allocator
add_library(thrust_allocator SHARED src/thrust_allocator.cpp)

# Link libraries and set dependencies
target_include_directories(thrust_allocator PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(thrust_allocator ${OpenCV_LIBS})
ament_target_dependencies(thrust_allocator rclcpp rclcpp_components std_msgs geometry_msgs OpenCV)

# Register the component with RCLCPP
rclcpp_components_register_node(thrust_allocator
    PLUGIN "john_controls::ThrustAllocator"
    EXECUTABLE thrust_allocator_node
)

# Install the shared library
install(TARGETS thrust_allocator
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

# Install header files
install(DIRECTORY include/
    DESTINATION include
)

# Install configuration files
install(DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config
)

# Install launch files
install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()