cmake_minimum_required(VERSION 3.5)
project(stdstates)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fsm REQUIRED)
find_package(drone_lib REQUIRED)

# Header-only library — export include directories
# When states need .cpp implementations, add them here:
# add_library(${PROJECT_NAME}
#   src/some_state.cpp
# )
# ament_target_dependencies(${PROJECT_NAME} rclcpp Eigen3 fsm drone_lib)

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

# Export the package
ament_export_include_directories(include)
ament_export_dependencies(rclcpp Eigen3 fsm drone_lib)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()