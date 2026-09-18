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
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rclpy REQUIRED)
find_package(Eigen3 REQUIRED)

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

install(FILES scripts/analytical_traj_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME analytical_traj_node
  PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE
              GROUP_READ GROUP_EXECUTE
              WORLD_READ WORLD_EXECUTE)

# ── C++ dynamics node (1 kHz RK4 integration, simulation only) ────────────────
add_executable(dynamics_node src/dynamics_node.cpp)
target_include_directories(dynamics_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(dynamics_node PUBLIC cxx_std_17)
ament_target_dependencies(dynamics_node rclcpp std_msgs)

# ── C++ kinematics node (50 Hz state post-processing, sim + real drone) ───────
add_executable(kinematics_node src/kinematics_node.cpp)
target_include_directories(kinematics_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(kinematics_node PUBLIC cxx_std_17)
ament_target_dependencies(kinematics_node rclcpp std_msgs geometry_msgs nav_msgs tf2_ros)

# ── C++ controller nodes (50 Hz) ─────────────────────────────────────────────
add_executable(lqr_controller src/lqr_controller.cpp)
target_include_directories(lqr_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(lqr_controller PUBLIC cxx_std_17)
ament_target_dependencies(lqr_controller rclcpp std_msgs)
target_link_libraries(lqr_controller Eigen3::Eigen)

add_executable(nonlinear_pd_controller src/nonlinear_pd_controller.cpp)
target_include_directories(nonlinear_pd_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(nonlinear_pd_controller PUBLIC cxx_std_17)
ament_target_dependencies(nonlinear_pd_controller rclcpp std_msgs)
target_link_libraries(nonlinear_pd_controller Eigen3::Eigen)

add_executable(sf_controller src/sf_controller.cpp)
target_include_directories(sf_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(sf_controller PUBLIC cxx_std_17)
ament_target_dependencies(sf_controller rclcpp std_msgs)
target_link_libraries(sf_controller Eigen3::Eigen)

# ── C++ sensor noise node (adds Gaussian noise to simulate real sensors) ─────
add_executable(sensor_noise_node src/sensor_noise_node.cpp)
target_include_directories(sensor_noise_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(sensor_noise_node PUBLIC cxx_std_17)
ament_target_dependencies(sensor_noise_node rclcpp std_msgs)

# ── C++ Kalman filter node (estimates pose + velocity from noisy measurements) ─
add_executable(kalman_filter_node src/kalman_filter_node.cpp)
target_include_directories(kalman_filter_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(kalman_filter_node PUBLIC cxx_std_17)
ament_target_dependencies(kalman_filter_node rclcpp std_msgs geometry_msgs)
target_link_libraries(kalman_filter_node Eigen3::Eigen)

add_executable(sf_mimo_controller src/sf_mimo_controller.cpp)
target_include_directories(sf_mimo_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(sf_mimo_controller PUBLIC cxx_std_17)
ament_target_dependencies(sf_mimo_controller rclcpp std_msgs)
target_link_libraries(sf_mimo_controller Eigen3::Eigen)

install(TARGETS dynamics_node kinematics_node lqr_controller nonlinear_pd_controller sf_controller sf_mimo_controller
        sensor_noise_node kalman_filter_node
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
