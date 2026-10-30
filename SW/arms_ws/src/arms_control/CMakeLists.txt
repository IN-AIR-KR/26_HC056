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
find_package(arms_msgs REQUIRED)

# -----------------------------------------------------------------------
# Library: state machine + PID
# -----------------------------------------------------------------------
add_library(arms_control_lib
  src/pid_controller.cpp
  src/pn_guidance.cpp
  src/state_machine.cpp
)

target_include_directories(arms_control_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(arms_control_lib
  rclcpp
  arms_msgs
)

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
  arms_msgs
)

if(BUILD_TESTING)
  add_executable(pn_guidance_smoke test/pn_guidance_smoke.cpp)
  target_link_libraries(pn_guidance_smoke arms_control_lib)
  add_test(NAME pn_guidance_smoke COMMAND pn_guidance_smoke)
  add_executable(state_machine_safety_test test/state_machine_safety_test.cpp)
  target_link_libraries(state_machine_safety_test arms_control_lib)
  add_test(NAME state_machine_safety_test COMMAND state_machine_safety_test)
  # 실제 노드 바이너리 + 가짜 입력으로 자동→수동 인계·안전 전이를 본다 (PX4/Gazebo 불필요).
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(control_handover_integration test/test_handover_integration.py
    TIMEOUT 120
    ENV
      ARMS_CONTROL_NODE_EXE=$<TARGET_FILE:arms_control_node>
      ARMS_CONTROL_PARAMS=${CMAKE_CURRENT_SOURCE_DIR}/config/control_params.yaml)
  ament_add_pytest_test(control_sim_time_integration test/test_sim_time_integration.py
    TIMEOUT 120
    ENV
      ARMS_CONTROL_NODE_EXE=$<TARGET_FILE:arms_control_node>
      ARMS_CONTROL_PARAMS=${CMAKE_CURRENT_SOURCE_DIR}/config/control_params.yaml)
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
