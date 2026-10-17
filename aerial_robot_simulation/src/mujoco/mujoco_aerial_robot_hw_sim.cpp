#include <aerial_robot_simulation/mujoco/mujoco_aerial_robot_hw_sim.h>
#include <map>
#include <regex>
#include <stdexcept>

namespace mujoco_ros::control
{
bool AerialRobotHWSim::InitSim(const mjModel* m_ptr, mjData* d_ptr, mujoco_ros::MujocoEnv* mujoco_env_ptr,
                               const std::string& robot_namespace, ros::NodeHandle model_nh,
                               const urdf::Model* const urdf_model,
                               std::vector<transmission_interface::TransmissionInfo> transmissions,
                               bool ignore_actuators)
{
  model_nh.param<std::string>("simulation/model_prefix", mujoco_model_prefix_, "");
  auto required_id = [&](mjtObj type, const std::string& name) {
    const std::string qualified_name = mujoco_model_prefix_ + name;
    const int id = mj_name2id(m_ptr, type, qualified_name.c_str());
    if (id < 0)
      throw std::runtime_error("Missing MuJoCo object: " + qualified_name);
    return id;
  };

  std::vector<transmission_interface::TransmissionInfo> standard_transmissions;
  std::map<int, int> rotors;
  try
  {
    fc_id_ = required_id(mjOBJ_SITE, "fc");
    const char* sensor_names[] = {"acc", "gyro", "mag"};
    for (int i = 0; i < 3; ++i)
    {
      sensor_ids_[i] = required_id(mjOBJ_SENSOR, sensor_names[i]);
      const int id = sensor_ids_[i];
      if (m_ptr->sensor_dim[id] != 3 || m_ptr->sensor_objtype[id] != mjOBJ_SITE ||
          m_ptr->sensor_objid[id] != fc_id_)
        throw std::runtime_error("MuJoCo IMU must have three axes and reference this robot's fc site");
    }

    // FlightControl indexes motors numerically; URDF transmission order is arbitrary.
    const std::regex rotor_pattern("rotor([1-9][0-9]*)");
    for (const auto& trans : transmissions)
    {
      bool is_rotor = false;
      for (const auto& actuator : trans.actuators_)
        for (const auto& hw_iface : actuator.hardware_interfaces_)
          if (hw_iface == "RotorInterface" || hw_iface == "hardware_interface/RotorInterface")
            is_rotor = true;
      for (const auto& joint : trans.joints_)
        for (const auto& hw_iface : joint.hardware_interfaces_)
          if (hw_iface == "RotorInterface" || hw_iface == "hardware_interface/RotorInterface")
            is_rotor = true;
      if (!is_rotor)
      {
        standard_transmissions.push_back(trans);
        continue;
      }
      if (trans.joints_.size() != 1 || trans.actuators_.size() != 1)
        throw std::runtime_error("A rotor transmission must have exactly one joint and actuator");
      const std::string& name = trans.joints_[0].name_;
      std::smatch match;
      if (!std::regex_match(name, match, rotor_pattern))
        throw std::runtime_error("MuJoCo rotor joint must be named rotor1..N: " + name);
      if (!rotors.emplace(std::stoi(match[1]), required_id(mjOBJ_ACTUATOR, name)).second)
        throw std::runtime_error("Duplicate MuJoCo rotor transmission: " + name);
    }
    rotor_actuator_ids_.clear();
    for (const auto& rotor : rotors)
    {
      if (rotor.first != rotor_actuator_ids_.size() + 1)
        throw std::runtime_error("MuJoCo rotors must be numbered contiguously from rotor1");
      rotor_actuator_ids_.push_back(rotor.second);
    }
    if (rotor_actuator_ids_.empty())
      throw std::runtime_error("No RotorInterface transmissions for " + model_nh.getNamespace());
  }
  catch (const std::exception& error)
  {
    ROS_ERROR_STREAM_NAMED("aerial_robot_hw_sim", error.what());
    return false;
  }

  if (!DefaultRobotHWSim::InitSim(m_ptr, d_ptr, mujoco_env_ptr, robot_namespace, model_nh, urdf_model,
                                 standard_transmissions, ignore_actuators))
    return false;

  int root_body = m_ptr_->site_bodyid[fc_id_];
  while (m_ptr_->body_parentid[root_body] != 0)
    root_body = m_ptr_->body_parentid[root_body];
  ROS_INFO_STREAM("[mujoco] " << model_nh.getNamespace() << " robot mass is "
                             << m_ptr_->body_subtreemass[root_body]);

  /* Initialize spinal interface */
  if (!spinal_interface_.init(model_nh, rotor_actuator_ids_.size()))
    return false;
  registerInterface(&spinal_interface_);

  ros::NodeHandle simulation_nh = ros::NodeHandle(model_nh, "simulation");
  simulation_nh.param("ground_truth_pub_rate", ground_truth_pub_rate_, 0.01);                               // [sec]
  simulation_nh.param("ground_truth_pos_noise", ground_truth_pos_noise_, 0.0);                              // m
  simulation_nh.param("ground_truth_vel_noise", ground_truth_vel_noise_, 0.0);                              // m/s
  simulation_nh.param("ground_truth_rot_noise", ground_truth_rot_noise_, 0.0);                              // rad
  simulation_nh.param("ground_truth_angular_noise", ground_truth_angular_noise_, 0.0);                      // rad/s
  simulation_nh.param("ground_truth_rot_drift", ground_truth_rot_drift_, 0.0);                              // rad
  simulation_nh.param("ground_truth_vel_drift", ground_truth_vel_drift_, 0.0);                              // m/s
  simulation_nh.param("ground_truth_angular_drift", ground_truth_angular_drift_, 0.0);                      // rad/s
  simulation_nh.param("ground_truth_rot_drift_frequency", ground_truth_rot_drift_frequency_, 0.0);          // 1/s
  simulation_nh.param("ground_truth_vel_drift_frequency", ground_truth_vel_drift_frequency_, 0.0);          // 1/s
  simulation_nh.param("ground_truth_angular_drift_frequency", ground_truth_angular_drift_frequency_, 0.0);  // 1/s

  simulation_nh.param("mocap_pub_rate", mocap_pub_rate_, 0.01);     // [sec]
  simulation_nh.param("mocap_pos_noise", mocap_pos_noise_, 0.001);  // m
  simulation_nh.param("mocap_rot_noise", mocap_rot_noise_, 0.001);  // rad
  ground_truth_pub_ = model_nh.advertise<nav_msgs::Odometry>("ground_truth", 1);
  mocap_pub_ = model_nh.advertise<geometry_msgs::PoseStamped>("mocap/pose", 1);

  return true;
}

void AerialRobotHWSim::ReadSim(ros::Time time, ros::Duration period)
{
  /* get the pose of the fc site in mujoco, and set it as the ground truth value for spinal interface */
  const int fc_id = fc_id_;
  if (time < last_ground_truth_time_ || time < last_mocap_time_)
  {
    last_ground_truth_time_ = ros::Time();
    last_mocap_time_ = ros::Time();
    for (int i = 0; i < 3; ++i)
    {
      ground_truth_rot_curr_drift_[i] = 0;
      ground_truth_vel_curr_drift_[i] = 0;
      ground_truth_angular_curr_drift_[i] = 0;
    }
  }
  mjtNum* site_xpos = d_ptr_->site_xpos;
  mjtNum* site_xmat = d_ptr_->site_xmat;
  tf::Matrix3x3 fc_rot_mat =
      tf::Matrix3x3(site_xmat[9 * fc_id + 0], site_xmat[9 * fc_id + 1], site_xmat[9 * fc_id + 2],
                    site_xmat[9 * fc_id + 3], site_xmat[9 * fc_id + 4], site_xmat[9 * fc_id + 5],
                    site_xmat[9 * fc_id + 6], site_xmat[9 * fc_id + 7], site_xmat[9 * fc_id + 8]);
  tf::Quaternion fc_quat;
  fc_rot_mat.getRotation(fc_quat);

  /* get the imu sensor data in mujoco, and set it as the input for spinal interface */
  tf::Vector3 values[3];
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      values[i][j] = d_ptr_->sensordata[m_ptr_->sensor_adr[sensor_ids_[i]] + j];
  const auto& acc = values[0];
  const auto& gyro = values[1];
  const auto& mag = values[2];

  spinal_interface_.setImuValue(acc.x(), acc.y(), acc.z(), gyro.x(), gyro.y(), gyro.z());
  spinal_interface_.setMagValue(mag.x(), mag.y(), mag.z());

  spinal_interface_.stateEstimate();

  // Match Gazebo's ground truth: world linear velocity and fc-frame angular velocity.
  nav_msgs::Odometry odom_msg;
  odom_msg.header.stamp = time;
  mjtNum fc_velocity[6];
  mj_objectVelocity(m_ptr_, d_ptr_, mjOBJ_SITE, fc_id_, fc_velocity, 0);
  tf::Vector3 position, velocity, angular, rotation_noise;
  for (int i = 0; i < 3; ++i)
  {
    position[i] = site_xpos[3 * fc_id + i] + gazebo::gaussianKernel(ground_truth_pos_noise_);
    rotation_noise[i] = gazebo::addNoise(ground_truth_rot_curr_drift_[i], ground_truth_rot_drift_,
                                        ground_truth_rot_drift_frequency_, 0, ground_truth_rot_noise_, period.toSec());
    velocity[i] = fc_velocity[3 + i] + gazebo::addNoise(ground_truth_vel_curr_drift_[i], ground_truth_vel_drift_,
                                                     ground_truth_vel_drift_frequency_, 0, ground_truth_vel_noise_, period.toSec());
    angular[i] = gyro[i] + gazebo::addNoise(ground_truth_angular_curr_drift_[i], ground_truth_angular_drift_,
                                          ground_truth_angular_drift_frequency_, 0, ground_truth_angular_noise_, period.toSec());
  }
  tf::Quaternion rotation_delta;
  rotation_delta.setRPY(rotation_noise.x(), rotation_noise.y(), rotation_noise.z());
  const tf::Quaternion noisy_rotation = fc_quat * rotation_delta;
  tf::pointTFToMsg(position, odom_msg.pose.pose.position);
  tf::quaternionTFToMsg(noisy_rotation, odom_msg.pose.pose.orientation);
  tf::vector3TFToMsg(velocity, odom_msg.twist.twist.linear);
  tf::vector3TFToMsg(angular, odom_msg.twist.twist.angular);

  spinal_interface_.setGroundTruthStates(noisy_rotation.x(), noisy_rotation.y(), noisy_rotation.z(), noisy_rotation.w(),
                                         angular.x(), angular.y(), angular.z());
  if ((time - last_ground_truth_time_).toSec() >= ground_truth_pub_rate_)
  {
    ground_truth_pub_.publish(odom_msg);
    last_ground_truth_time_ = time;
  }

  if ((time - last_mocap_time_).toSec() >= mocap_pub_rate_)
  {
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = time;
    pose_msg.pose.position.x = site_xpos[3 * fc_id + 0] + gazebo::gaussianKernel(mocap_pos_noise_);
    pose_msg.pose.position.y = site_xpos[3 * fc_id + 1] + gazebo::gaussianKernel(mocap_pos_noise_);
    pose_msg.pose.position.z = site_xpos[3 * fc_id + 2] + gazebo::gaussianKernel(mocap_pos_noise_);

    tf::Quaternion q_delta;
    q_delta.setRPY(gazebo::gaussianKernel(mocap_rot_noise_), gazebo::gaussianKernel(mocap_rot_noise_),
                   gazebo::gaussianKernel(mocap_rot_noise_));
    tf::Quaternion q_noise = fc_quat * q_delta;
    pose_msg.pose.orientation.x = q_noise.x();
    pose_msg.pose.orientation.y = q_noise.y();
    pose_msg.pose.orientation.z = q_noise.z();
    pose_msg.pose.orientation.w = q_noise.w();

    mocap_pub_.publish(pose_msg);
    last_mocap_time_ = time;
  }

  DefaultRobotHWSim::ReadSim(time, period);
}

void AerialRobotHWSim::WriteSim(ros::Time time, ros::Duration period)
{
  // normal joints control by default robot hw sim
  DefaultRobotHWSim::WriteSim(time, period);

  // set rotor force got from spinal interface to mujoco actuator
  for (int i = 0; i < spinal_interface_.getMotorNum(); i++)
  {
    const int rotor_id = rotor_actuator_ids_.at(i);
    double rotor_force = spinal_interface_.getForce(i);
    if (std::isfinite(rotor_force))
      d_ptr_->ctrl[rotor_id] = rotor_force;
  }
}

}  // namespace mujoco_ros::control

PLUGINLIB_EXPORT_CLASS(mujoco_ros::control::AerialRobotHWSim, mujoco_ros::control::RobotHWSim)
