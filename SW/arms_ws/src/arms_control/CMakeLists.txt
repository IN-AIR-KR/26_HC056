cmake_minimum_required(VERSION 3.14)
project(arms_control)

set(CMAKE_CXX_STANDARD 17)

# -----------------------------------------------------------------------
# Dependencies
# -----------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(arms_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# -----------------------------------------------------------------------
# Library: state machine + PID
# -----------------------------------------------------------------------
add_library(arms_control_lib
  src/pid_controller.cpp
  src/pn_guidance.cpp
  src/state_machine.cpp
  src/yang_dkf.cpp
)

target_include_directories(arms_control_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(arms_control_lib
  rclcpp
  arms_msgs
)
target_link_libraries(arms_control_lib Eigen3::Eigen)

# -----------------------------------------------------------------------
# Executable: arms_control_node
# -----------------------------------------------------------------------
add_executable(arms_control_node src/arms_control_node.cpp)

target_link_libraries(arms_control_node
  arms_control_lib
)

ament_target_dependencies(arms_control_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  arms_msgs
)

if(BUILD_TESTING)
  add_executable(pn_guidance_smoke test/pn_guidance_smoke.cpp)
  target_link_libraries(pn_guidance_smoke arms_control_lib)
  add_test(NAME pn_guidance_smoke COMMAND pn_guidance_smoke)
endif()

# -----------------------------------------------------------------------
# Install
# -----------------------------------------------------------------------
install(TARGETS arms_control_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# -----------------------------------------------------------------------
ament_package()
