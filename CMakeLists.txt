cmake_minimum_required(VERSION 3.8)
project(bt_pkg)

# Set C++ Standard (Standard for ROS 2 Humble/Iron is C++17)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ==========================================
# 1. FIND DEPENDENCIES
# ==========================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(yolo11_seg_interfaces REQUIRED)
find_package(nav2_behavior_tree REQUIRED)

# ==========================================
# 2. BUILD THE EXECUTABLE
# ==========================================
# We define one main executable "bt_manager" that contains all your custom logic.
# Ensure these .cpp files exist in your 'src' folder.
add_executable(bt_manager
  src/main.cpp
  src/read_json.cpp
  src/check_goal_seen.cpp
  src/navigate_to_pose.cpp
  src/find_approach_pose.cpp
  src/rotate_360.cpp
)

# Link the executable to the found libraries
ament_target_dependencies(bt_manager
  rclcpp
  behaviortree_cpp
  rclcpp_action
  geometry_msgs
  nav_msgs
  nav2_msgs
  nlohmann_json
  yolo11_seg_interfaces
)

# Tell the compiler where the header (.hpp) files are located
target_include_directories(bt_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ==========================================
# 3. INSTALL RULES
# ==========================================
# These rules ensure that when you run 'colcon build', the necessary files
# are copied to the install directory so ROS can find them.

# 1. Install the executable
install(TARGETS bt_manager
  DESTINATION lib/${PROJECT_NAME}
)

# 2. Install the Behavior Tree XML files
install(DIRECTORY bt_xml
  DESTINATION share/${PROJECT_NAME}
)

# ==========================================
# 4. TESTING (Standard Boilerplate)
# ==========================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()