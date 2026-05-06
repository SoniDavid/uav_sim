cmake_minimum_required(VERSION 3.8)
project(hexa_tilt_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -O3)
endif()

#  Dependencies 
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rclpy REQUIRED)
find_package(Eigen3 REQUIRED)

#  Python library (hexa_tilt_sim_scripts) 
ament_python_install_package(hexa_tilt_sim_scripts PACKAGE_DIR scripts)

#  Python executables 
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

#  C++ dynamics node (1 kHz, simulation) 
add_executable(dynamics_node src/dynamics_node.cpp)
target_include_directories(dynamics_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(dynamics_node PUBLIC cxx_std_17)
ament_target_dependencies(dynamics_node rclcpp std_msgs)

#  C++ kinematics node (50 Hz) 
add_executable(kinematics_node src/kinematics_node.cpp)
target_include_directories(kinematics_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(kinematics_node PUBLIC cxx_std_17)
ament_target_dependencies(kinematics_node rclcpp std_msgs geometry_msgs nav_msgs tf2_ros)

#  C++ FxT-PD controller node (250 Hz) 
add_executable(fxt_pd_controller src/fxt_pd_controller.cpp)
target_include_directories(fxt_pd_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(fxt_pd_controller PUBLIC cxx_std_17)
ament_target_dependencies(fxt_pd_controller rclcpp std_msgs)

#  C++ Simple-PD controller node (250 Hz, no FxT differentiator) 
add_executable(simple_pd_controller src/simple_pd_controller.cpp)
target_include_directories(simple_pd_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(simple_pd_controller PUBLIC cxx_std_17)
ament_target_dependencies(simple_pd_controller rclcpp std_msgs)

install(TARGETS dynamics_node kinematics_node fxt_pd_controller simple_pd_controller
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

#  Shared data 
install(DIRECTORY launch config meshes urdf
  DESTINATION share/${PROJECT_NAME})
install(FILES resource/hexa_tilt_sim
  DESTINATION share/ament_index/resource_index/packages)
install(FILES package.xml
  DESTINATION share/${PROJECT_NAME})

ament_package()
