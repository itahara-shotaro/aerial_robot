cmake_minimum_required(VERSION 3.0.2)
project(aerial_robot_simulation)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  aerial_robot_base
  aerial_robot_estimation
  aerial_robot_msgs
  aerial_robot_model
  gazebo_ros_control
  controller_manager
  kdl_parser
  mujoco_ros_control
  roscpp
  spinal
  tf
  )

# Depend on system install of Gazebo
find_package(GAZEBO REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(urdfdom_headers REQUIRED)
include_directories(${orocos_kdl_INCLUDE_DIRS} ${urdfdom_headers_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES aerial_robot_hw_sim spinal_interface flight_controllers
  CATKIN_DEPENDS aerial_robot_base aerial_robot_estimation aerial_robot_msgs aerial_robot_model gazebo_ros_control kdl_parser roscpp spinal tf
  DEPENDS GAZEBO orocos_kdl urdfdom_headers
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  include
  )


add_definitions(-DSIMULATION)

add_library(flight_controllers
  src/mujoco/mujoco_attitude_controller.cpp
  src/simulation_attitude_controller.cpp)
add_dependencies(flight_controllers spinal_generate_messages_cpp)
target_link_libraries(flight_controllers ${catkin_LIBRARIES})

add_library(spinal_interface src/spinal_interface.cpp)
target_link_libraries(spinal_interface ${catkin_LIBRARIES})

add_library(mujoco_spinal_interface src/mujoco/mujoco_spinal_interface.cpp)
target_link_libraries(mujoco_spinal_interface ${catkin_LIBRARIES})

add_library(aerial_robot_hw_sim src/aerial_robot_hw_sim.cpp)
target_link_libraries(aerial_robot_hw_sim spinal_interface ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${orocos_kdl_LIBRARIES})

add_library(mujoco_aerial_robot_hw_sim src/mujoco/mujoco_aerial_robot_hw_sim.cpp
  src/mujoco/mujoco_multi_robot_hw_sim.cpp)
target_link_libraries(mujoco_aerial_robot_hw_sim ${catkin_LIBRARIES} mujoco_spinal_interface)


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS aerial_robot_hw_sim spinal_interface flight_controllers mujoco_spinal_interface mujoco_aerial_robot_hw_sim
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

install(FILES aerial_robot_hw_sim_plugins.xml flight_controllers_plugins.xml mujoco_robot_hw_sim_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/merge_mujoco_models.py scripts/multi_mujoco.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(FILES README.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_mujoco_scene.py)
  catkin_add_gtest(test_mujoco_estimator test/test_mujoco_estimator.cpp)
  if(TARGET test_mujoco_estimator)
    target_link_libraries(test_mujoco_estimator ${catkin_LIBRARIES})
  endif()
  add_executable(mujoco_scene_physics_test test/mujoco_scene_physics_test.cpp)
  target_link_libraries(mujoco_scene_physics_test ${catkin_LIBRARIES})
endif()
