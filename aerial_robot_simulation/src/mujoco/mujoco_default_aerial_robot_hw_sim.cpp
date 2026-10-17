#include <aerial_robot_simulation/mujoco/mujoco_default_aerial_robot_hw_sim.h>
#include <set>

namespace mujoco_ros::control
{

bool DefaultAerialRobotHWSim::InitSim(const mjModel* m_ptr, mjData* d_ptr, mujoco_ros::MujocoEnv* mujoco_env_ptr,
                                      const std::string& robot_namespace, ros::NodeHandle model_nh,
                                      const urdf::Model* const urdf_model,
                                      std::vector<transmission_interface::TransmissionInfo> transmissions,
                                      bool ignore_actuators)
{
  model_nh.param<std::string>("simulation/model_prefix", mujoco_model_prefix_, "");
  if (!urdf_model || !urdf_model->getRoot())
  {
    ROS_ERROR("MuJoCo aerial robot requires a URDF root link");
    return false;
  }
  root_link_name_ = urdf_model->getRoot()->name;
  const int root_body_id = mj_name2id(m_ptr, mjOBJ_BODY, (mujoco_model_prefix_ + root_link_name_).c_str());
  if (root_body_id < 0 || m_ptr->body_jntnum[root_body_id] != 1 ||
      m_ptr->jnt_type[m_ptr->body_jntadr[root_body_id]] != mjJNT_FREE)
  {
    ROS_ERROR_STREAM("MuJoCo robot root must have one free joint: " << mujoco_model_prefix_ + root_link_name_);
    return false;
  }
  const int root_joint_id = m_ptr->body_jntadr[root_body_id];
  root_qpos_adr_ = m_ptr->jnt_qposadr[root_joint_id];
  root_dof_adr_ = m_ptr->jnt_dofadr[root_joint_id];

  std::vector<transmission_interface::TransmissionInfo> standard_transmissions;
  std::vector<transmission_interface::TransmissionInfo> rotor_transmissions;

  std::set<std::string> rotor_joints;
  /* separate rotor transmissions and standard transmissions */
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
    if (is_rotor)
    {
      if (trans.joints_.size() != 1 || trans.actuators_.size() != 1)
      {
        ROS_ERROR("A rotor transmission must have exactly one joint and actuator");
        return false;
      }
      if (!rotor_joints.insert(trans.joints_[0].name_).second)
      {
        ROS_ERROR_STREAM("Duplicate MuJoCo rotor transmission: " << trans.joints_[0].name_);
        return false;
      }
      rotor_transmissions.push_back(trans);
    }
    else
      standard_transmissions.push_back(trans);
  }

  /* Initialize standard joint handlers */
  if (!DefaultRobotHWSim::InitSim(m_ptr, d_ptr, mujoco_env_ptr, robot_namespace, model_nh, urdf_model,
                                 standard_transmissions, ignore_actuators))
    return false;

  /* Initialize rotor handlers */
  size_t num_rotors = rotor_transmissions.size();
  rotor_names_.resize(num_rotors);
  rotor_actuator_ids_.resize(num_rotors);
  rotor_cmd_.resize(num_rotors);
  rotor_pos_.resize(num_rotors);
  rotor_vel_.resize(num_rotors);
  rotor_eff_.resize(num_rotors);
  for (size_t i = 0; i < num_rotors; i++)
  {
    const auto& transmission = rotor_transmissions[i];

    std::string joint_name = transmission.joints_[0].name_;

    const std::string actuator_name = mujoco_model_prefix_ + joint_name;
    const int actuator_id = mj_name2id(m_ptr_, mjOBJ_ACTUATOR, actuator_name.c_str());
    if (actuator_id < 0)
    {
      ROS_ERROR_STREAM("Missing MuJoCo rotor actuator: " << actuator_name);
      return false;
    }
    rotor_actuator_ids_.at(i) = actuator_id;
    rotor_names_.at(i) = joint_name;
    rotor_cmd_.at(i) = 0.0;
    rotor_pos_.at(i) = 0.0;
    rotor_vel_.at(i) = 0.0;
    rotor_eff_.at(i) = 0.0;

    /* create handle for each rotor */
    hardware_interface::JointStateHandle state_handle(rotor_names_.at(i), &rotor_pos_.at(i), &rotor_vel_.at(i),
                                                      &rotor_eff_.at(i));
    hardware_interface::JointHandle rotor_handle(state_handle, &rotor_cmd_.at(i));

    rotor_interface_.registerHandle(rotor_handle);
  }

  registerInterface(&rotor_interface_);

  robot_ns_ = model_nh.getNamespace();
  odom_pub_ = model_nh.advertise<nav_msgs::Odometry>("odom", 10);

  ros::NodeHandle simulation_nh = ros::NodeHandle(model_nh, "simulation");
  simulation_nh.param("odom_pub_rate", odom_pub_rate_, 0.01);          // [sec]
  simulation_nh.param("tf_broadcast_rate", tf_broadcast_rate_, 0.01);  // [sec]
  simulation_nh.param("publish_odom", publish_odom_, true);
  simulation_nh.param("publish_tf", publish_tf_, false);
  simulation_nh.param("root_link_name", root_link_name_,
                      root_link_name_);  // overwrite root link name if specified in yaml
  odom_pub_last_time_ = 0.0;
  tf_broadcast_last_time_ = 0.0;

  return true;
}

void DefaultAerialRobotHWSim::ReadSim(ros::Time time, ros::Duration period)
{
  DefaultRobotHWSim::ReadSim(time, period);

  // reset last publish time after simulator reset
  if (ros::Time(time).toSec() < odom_pub_last_time_ || ros::Time(time).toSec() < tf_broadcast_last_time_)
  {
    odom_pub_last_time_ = 0.0;
    tf_broadcast_last_time_ = 0.0;
  }

  if (publish_odom_ && (ros::Time(time).toSec() - odom_pub_last_time_ >= odom_pub_rate_))
  {
    publishOdometry(time);
    odom_pub_last_time_ = ros::Time(time).toSec();
  }

  if (publish_tf_ && (ros::Time(time).toSec() - tf_broadcast_last_time_ >= tf_broadcast_rate_))
  {
    publishTF(time);
    tf_broadcast_last_time_ = ros::Time(time).toSec();
  }
}

void DefaultAerialRobotHWSim::WriteSim(ros::Time time, ros::Duration period)
{
  DefaultRobotHWSim::WriteSim(time, period);

  /* rotor part */
  for (size_t i = 0; i < rotor_names_.size(); i++)
  {
    const int rotor_id = rotor_actuator_ids_.at(i);
    if (std::isfinite(rotor_cmd_.at(i)))
      d_ptr_->ctrl[rotor_id] = rotor_cmd_.at(i);
  }
}

void DefaultAerialRobotHWSim::publishOdometry(ros::Time time)
{
  nav_msgs::Odometry odom_msg;
  const mjtNum* qpos = d_ptr_->qpos + root_qpos_adr_;
  const mjtNum* qvel = d_ptr_->qvel + root_dof_adr_;
  tf::Vector3 v_global(qvel[0], qvel[1], qvel[2]);
  tf::Quaternion q(qpos[4], qpos[5], qpos[6], qpos[3]);
  tf::Vector3 v_local = tf::quatRotate(q.inverse(), v_global);

  odom_msg.header.stamp = time;
  odom_msg.header.frame_id = "world";
  odom_msg.child_frame_id = tf::resolve(robot_ns_, root_link_name_);
  odom_msg.pose.pose.position.x = qpos[0];
  odom_msg.pose.pose.position.y = qpos[1];
  odom_msg.pose.pose.position.z = qpos[2];
  odom_msg.pose.pose.orientation.w = qpos[3];
  odom_msg.pose.pose.orientation.x = qpos[4];
  odom_msg.pose.pose.orientation.y = qpos[5];
  odom_msg.pose.pose.orientation.z = qpos[6];
  odom_msg.twist.twist.linear.x = v_local.x();
  odom_msg.twist.twist.linear.y = v_local.y();
  odom_msg.twist.twist.linear.z = v_local.z();
  odom_msg.twist.twist.angular.x = qvel[3];
  odom_msg.twist.twist.angular.y = qvel[4];
  odom_msg.twist.twist.angular.z = qvel[5];
  odom_pub_.publish(odom_msg);
}

void DefaultAerialRobotHWSim::publishTF(ros::Time time)
{
  geometry_msgs::TransformStamped root_pose_transform;
  const mjtNum* qpos = d_ptr_->qpos + root_qpos_adr_;
  root_pose_transform.header.stamp = time;
  root_pose_transform.header.frame_id = "world";
  root_pose_transform.child_frame_id = tf::resolve(robot_ns_, root_link_name_);
  root_pose_transform.transform.translation.x = qpos[0];
  root_pose_transform.transform.translation.y = qpos[1];
  root_pose_transform.transform.translation.z = qpos[2];
  root_pose_transform.transform.rotation.w = qpos[3];
  root_pose_transform.transform.rotation.x = qpos[4];
  root_pose_transform.transform.rotation.y = qpos[5];
  root_pose_transform.transform.rotation.z = qpos[6];
  tf_broadcaster_.sendTransform(root_pose_transform);
}

}  // namespace mujoco_ros::control

PLUGINLIB_EXPORT_CLASS(mujoco_ros::control::DefaultAerialRobotHWSim, mujoco_ros::control::RobotHWSim)
