cmake_minimum_required(VERSION 3.8)
project(vtol_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Enable C++20 to support designated initializers
add_compile_options(-std=c++20)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)  # ROS 2 core library
find_package(px4_msgs REQUIRED)  # PX4 message definitions
find_package(tf2 REQUIRED)  # Added for quaternion to Euler conversion
find_package(tf2_ros REQUIRED)  # Added for TF2 ROS integration
find_package(Eigen3 REQUIRED)  # Find Eigen3 package

# Add include directories
include_directories(${EIGEN3_INCLUDE_DIRS})

# Define the executable
# vtol
# add_executable(actuator_test /src/vtol/actuator_test.cpp) 
add_executable(display_altitude src/vtol/display_altitude.cpp)

# Fixed Wing
add_executable(altitude src/fixed_wing/altitude.cpp)
add_executable(attitude src/fixed_wing/attitude.cpp)
add_executable(asmc_attitude src/fixed_wing/asmc_attitude.cpp)

add_executable(pid src/vtol/pid.cpp)


# Link libraries
target_link_libraries(display_altitude rclcpp::rclcpp ${tf2_LIBRARIES})  
target_link_libraries(pid rclcpp::rclcpp ${tf2_LIBRARIES} Eigen3::Eigen)  # Link Eigen3 to the vtol_controller target


# Declare dependencies
ament_target_dependencies(display_altitude rclcpp px4_msgs tf2 tf2_ros)
ament_target_dependencies(pid rclcpp px4_msgs tf2 tf2_ros)

ament_target_dependencies(altitude rclcpp px4_msgs tf2 tf2_ros)

ament_target_dependencies(attitude rclcpp px4_msgs tf2 tf2_ros)

ament_target_dependencies(asmc_attitude rclcpp px4_msgs tf2 tf2_ros)




# Install the executables
install(TARGETS 
  altitude
  attitude
  asmc_attitude
  display_altitude
  pid
  DESTINATION lib/${PROJECT_NAME})

# Final ROS 2 package declaration
ament_package()
