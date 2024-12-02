cmake_minimum_required(VERSION 3.8)
project(hello_moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Define Executables
add_executable(hello_moveit src/hello_moveit.cpp)
add_executable(talker src/publisher_lambda_function.cpp)

# Target dependencies
ament_target_dependencies(
  hello_moveit
  "moveit_ros_planning_interface"
  "rclcpp"
  "sensor_msgs"
)

ament_target_dependencies(
  talker
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
)

# Include directories for hello_moveit
target_include_directories(hello_moveit PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

# Include directories for talker
target_include_directories(talker PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

# Compile features
target_compile_features(hello_moveit PUBLIC c_std_99 cxx_std_17)
target_compile_features(talker PUBLIC c_std_99 cxx_std_17)

# Install Targets
install(TARGETS hello_moveit talker
  DESTINATION lib/${PROJECT_NAME})

# Enable Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skip linter checks for now
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
