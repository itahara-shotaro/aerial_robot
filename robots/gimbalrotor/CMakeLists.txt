cmake_minimum_required(VERSION 3.0.2)
project(gimbalrotor)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  aerial_robot_control
  aerial_robot_estimation
  aerial_robot_model
  aerial_robot_msgs
  spinal
  pluginlib
  roscpp
  rospy
  sensor_msgs
  geometry_msgs
  tf2_ros
  )

find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS aerial_robot_control aerial_robot_estimation aerial_robot_model aerial_robot_msgs geometry_msgs pluginlib roscpp rospy sensor_msgs spinal tf2_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

add_library(gimbalrotor_controller src/control/gimbalrotor_controller.cpp)
target_link_libraries(gimbalrotor_controller ${catkin_LIBRARIES})

add_library(gimbalrotor_robot_model src/model/gimbalrotor_robot_model.cpp)
target_link_libraries(gimbalrotor_robot_model ${catkin_LIBRARIES})

add_library(gimbalrotor_navigation src/gimbalrotor_navigation.cpp)
target_link_libraries(gimbalrotor_navigation ${catkin_LIBRARIES})

catkin_install_python(
  PROGRAMS scripts/trajectory_command/csv_trajectory_command.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  FILES
    scripts/trajectory_command/trajectory_command.launch
    scripts/trajectory_command/trajectory_command.yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts/trajectory_command
  )

install(
  DIRECTORY scripts/trajectory_command/trajectory
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts/trajectory_command
  )


install(DIRECTORY config plugins launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  )

# Keep the map-ICP launch, configuration, and pinned Python requirements
# together in the installed package as they are in the source tree.
install(DIRECTORY scripts/map_icp/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts/map_icp
  USE_SOURCE_PERMISSIONS
  )

catkin_install_python(PROGRAMS
  scripts/map_icp/multi_map_icp_alignment.py
  test/mocap_relative_tf.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_subdirectory(test)
endif()
