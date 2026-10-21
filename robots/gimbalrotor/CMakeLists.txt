cmake_minimum_required(VERSION 3.0.2)
project(gimbalrotor)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  aerial_robot_control
  aerial_robot_estimation
  aerial_robot_model
  aerial_robot_msgs
  spinal
  mujoco_ros_utils
  pluginlib
  roscpp
  )

find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS aerial_robot_control aerial_robot_estimation aerial_robot_model aerial_robot_msgs pluginlib roscpp spinal
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

add_library(gimbalrotor_controller src/control/gimbalrotor_controller.cpp src/control/lqi_gimbal_control.cpp)
target_link_libraries(gimbalrotor_controller ${catkin_LIBRARIES})

add_library(gimbalrotor_robot_model src/model/gimbalrotor_robot_model.cpp)
target_link_libraries(gimbalrotor_robot_model ${catkin_LIBRARIES})

add_library(gimbalrotor_navigation src/gimbalrotor_navigation.cpp)
target_link_libraries(gimbalrotor_navigation ${catkin_LIBRARIES})

install(DIRECTORY config plugins launch scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  )

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_subdirectory(test)
endif()

# Generate the upstream MuJoCo models, then select this robot's drag-free world.
mujoco_model_convert(${PROJECT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/config/mujoco_model.yaml)

# generate_mujoco_model is not built on aarch64 (vim4)
if(TARGET generate_mujoco_model)
  get_filename_component(GIMBALROTOR_MUJOCO_WORLD
    "${PROJECT_SOURCE_DIR}/../../aerial_robot_simulation/config/mujoco/world.xml" REALPATH)

  add_custom_target(configure_gimbalrotor_mujoco ALL
    COMMAND ${PYTHON_EXECUTABLE} ${PROJECT_SOURCE_DIR}/scripts/configure_mujoco_models.py
      ${PROJECT_SOURCE_DIR} ${GIMBALROTOR_MUJOCO_WORLD}
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})

  add_dependencies(configure_gimbalrotor_mujoco generate_mujoco_model)
endif()