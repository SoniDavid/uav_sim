cmake_minimum_required(VERSION 3.8)
project(quadrotor_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -O3)
endif()

# ── Dependencies ──────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rclpy REQUIRED)

# ── Python library (quadrotor_sim_scripts) ────────────────────────────────────
# Installs scripts/ directory as the "quadrotor_sim_scripts" Python package
ament_python_install_package(quadrotor_sim_scripts PACKAGE_DIR scripts)

# ── Python executables ────────────────────────────────────────────────────────
install(FILES scripts/reference_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME reference_node
  PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE
              GROUP_READ GROUP_EXECUTE
              WORLD_READ WORLD_EXECUTE)

install(FILES scripts/trajectory_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME trajectory_runner
  PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE
              GROUP_READ GROUP_EXECUTE
              WORLD_READ WORLD_EXECUTE)

# ── C++ dynamics node (1 kHz RK4 integration) ─────────────────────────────────
add_executable(dynamics_node src/dynamics_node.cpp)
target_include_directories(dynamics_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(dynamics_node PUBLIC cxx_std_17)
ament_target_dependencies(dynamics_node rclcpp std_msgs geometry_msgs tf2_ros)

# ── C++ controller node (event-driven, responds at 1 kHz) ────────────────────
add_executable(controller_node src/controller_node.cpp)
target_include_directories(controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(controller_node PUBLIC cxx_std_17)
ament_target_dependencies(controller_node rclcpp std_msgs)

install(TARGETS dynamics_node controller_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

# ── Shared data ───────────────────────────────────────────────────────────────
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})
install(FILES resource/quadrotor_sim
  DESTINATION share/ament_index/resource_index/packages)
install(FILES package.xml
  DESTINATION share/${PROJECT_NAME})

ament_package()
