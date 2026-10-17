#pragma once

#include <aerial_robot_simulation/mujoco/mujoco_spinal_interface.h>
#include <aerial_robot_simulation/noise_model.h>
#include <geometry_msgs/PoseStamped.h>
#include <mujoco_ros_control/default_robot_hw_sim.hpp>
#include <nav_msgs/Odometry.h>

namespace mujoco_ros::control
{
class AerialRobotHWSim : public mujoco_ros::control::DefaultRobotHWSim
{
public:
  AerialRobotHWSim(){};
  ~AerialRobotHWSim()
  {
  }

  bool InitSim(const mjModel* m_ptr, mjData* d_ptr, mujoco_ros::MujocoEnv* mujoco_env_ptr,
               const std::string& robot_namespace, ros::NodeHandle model_nh, const urdf::Model* const urdf_model,
               std::vector<transmission_interface::TransmissionInfo> transmissions, bool ignore_actuators = false) override;

  void ReadSim(ros::Time time, ros::Duration period) override;

  void WriteSim(ros::Time time, ros::Duration period) override;

protected:
  hardware_interface::MujocoSpinalInterface spinal_interface_;

  std::string mujoco_model_prefix_;
  int fc_id_ = -1;
  int sensor_ids_[3] = {-1, -1, -1};
  std::vector<int> rotor_actuator_ids_;
  ros::Publisher ground_truth_pub_;
  ros::Publisher mocap_pub_;
  double ground_truth_pub_rate_;
  double mocap_pub_rate_;
  double mocap_rot_noise_, mocap_pos_noise_;
  double ground_truth_pos_noise_, ground_truth_vel_noise_, ground_truth_rot_noise_, ground_truth_angular_noise_;
  double ground_truth_rot_drift_, ground_truth_vel_drift_, ground_truth_angular_drift_;
  double ground_truth_rot_drift_frequency_, ground_truth_vel_drift_frequency_, ground_truth_angular_drift_frequency_;
  double ground_truth_rot_curr_drift_[3] = {};
  double ground_truth_vel_curr_drift_[3] = {};
  double ground_truth_angular_curr_drift_[3] = {};

  ros::Time last_ground_truth_time_, last_mocap_time_;
};
}  // namespace mujoco_ros_control
