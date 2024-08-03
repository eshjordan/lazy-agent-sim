cmake_minimum_required(VERSION 3.8)
project(lazy_agent_sim)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Enable additional compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Build messages
ament_auto_generate_code()

# Export the plugin configuration file
pluginlib_export_plugin_description_file(webots_ros2_driver my_robot_driver.xml)

# Build the library
ament_auto_add_library(lazy_agent_sim SHARED NO_TARGET_LINK_LIBRARIES
  src/LazyAgentSim.cpp
)

# Build the executable
ament_auto_add_executable(obstacle_avoider NO_TARGET_LINK_LIBRARIES
  src/ObstacleAvoider.cpp
)

# Test
if(BUILD_TESTING)
  ament_auto_find_test_dependencies()

  set(AMENT_LINT_AUTO_FILE_EXCLUDE
    CONTRIBUTING.md
  )

  ament_lint_auto_find_test_dependencies()
endif()

# Install
ament_auto_package(
  INSTALL_TO_SHARE
    launch
    resource
    worlds
)
