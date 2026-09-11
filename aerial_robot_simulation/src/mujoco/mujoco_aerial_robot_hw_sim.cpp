#include <aerial_robot_simulation/mujoco/mujoco_aerial_robot_hw_sim.h>
#include <aerial_robot_simulation/noise_model.h>
#include <algorithm>
#include <map>
#include <regex>
#include <stdexcept>

namespace mujoco_ros_control
{

  bool AerialRobotHWSim::init(const std::string& robot_namespace,
                              ros::NodeHandle model_nh,
                              mjModel* mujoco_model,
                              mjData* mujoco_data
                              )
  {
    // The default interface owns every actuator in the world. Build explicit
    // per-robot maps instead, leaving ROS joint names local to each URDF.
    model_nh_ = model_nh;
    mujoco_model_ = mujoco_model;
    mujoco_data_ = mujoco_data;
    model_nh.param<std::string>("simulation/model_prefix", model_prefix_, "");
    auto owns = [this](const char* name) {
      return name && std::string(name).compare(0, model_prefix_.size(), model_prefix_) == 0;
    };
    auto required_id = [this](mjtObj type, const std::string& name) {
      int id = mj_name2id(mujoco_model_, type, (model_prefix_ + name).c_str());
      if (id < 0) throw std::runtime_error("Missing MuJoCo object: " + model_prefix_ + name);
      return id;
    };
    fc_id_ = required_id(mjOBJ_SITE, "fc");
    const char* sensor_names[] = {"acc", "gyro", "mag"};
    for (int i = 0; i < 3; ++i)
      {
        sensor_ids_[i] = required_id(mjOBJ_SENSOR, sensor_names[i]);
        if (mujoco_model_->sensor_dim[sensor_ids_[i]] != 3 ||
            mujoco_model_->sensor_objtype[sensor_ids_[i]] != mjOBJ_SITE ||
            mujoco_model_->sensor_objid[sensor_ids_[i]] != fc_id_)
          throw std::runtime_error("MuJoCo IMU must have three axes and reference this robot's fc site");
      }

    actuator_ids_.clear();
    joint_ids_.clear();
    joint_list_.clear();
    rotor_list_.clear();
    control_input_.assign(mujoco_model_->nu, 0.0);
    std::map<int, std::string> rotors;
    const std::regex rotor_pattern("rotor([1-9][0-9]*)");
    for (int i = 0; i < mujoco_model_->nu; ++i)
      {
        const char* name = mj_id2name(mujoco_model_, mjOBJ_ACTUATOR, i);
        if (!owns(name)) continue;
        actuator_ids_.push_back(i);
        std::string local_name = std::string(name).substr(model_prefix_.size());
        std::smatch match;
        if (std::regex_match(local_name, match, rotor_pattern))
          rotors.emplace(std::stoi(match[1]), name);
      }
    for (const auto& rotor : rotors)
      {
        if (rotor.first != rotor_list_.size() + 1)
          throw std::runtime_error("MuJoCo rotors must be numbered contiguously from rotor1");
        rotor_list_.push_back(rotor.second);
      }
    if (rotor_list_.empty()) throw std::runtime_error("No rotors for " + model_nh.getNamespace());
    for (int i = 0; i < mujoco_model_->njnt; ++i)
      {
        const char* name = mj_id2name(mujoco_model_, mjOBJ_JOINT, i);
        if (mujoco_model_->jnt_type[i] > mjJNT_BALL && owns(name))
          {
            joint_ids_.push_back(i);
            joint_list_.push_back(std::string(name).substr(model_prefix_.size()));
          }
      }
    joint_state_pub_ = model_nh.advertise<sensor_msgs::JointState>("joint_states", 1);
    control_input_sub_ = model_nh.subscribe("mujoco/ctrl_input", 1, &AerialRobotHWSim::controlInputCallback, this);

    // init joints from rosparam
    XmlRpc::XmlRpcValue all_servos_params;
    model_nh.getParam("servo_controller", all_servos_params);
    std::string init_value_param_name = "init_value";
    if (all_servos_params.getType() == XmlRpc::XmlRpcValue::TypeStruct)
    for(auto servo_group_params: all_servos_params)
      {
        if (servo_group_params.second.getType() != XmlRpc::XmlRpcValue::TypeStruct)
          continue;
        for(auto servo_params : servo_group_params.second)
          {
            if(servo_params.first.find("controller") != string::npos)
              {
                std::string servo_name = static_cast<std::string>(servo_params.second["name"]);
                double init_value = 0.0;

                // check simulation param exists
                if(!servo_group_params.second.hasMember("simulation") &&
                   !servo_params.second.hasMember("simulation"))
                  {
                    ROS_ERROR("please set mujoco servo parameters for %s, using sub namespace 'simulation:'", string(servo_params.second["name"]).c_str());
                    continue;
                  }

                // search init_value in servo params
                if(!servo_params.second.hasMember("simulation") ||
                   (servo_params.second.hasMember("simulation") && !servo_params.second["simulation"].hasMember(init_value_param_name)))
                  {
                    // search init_value in servo group params
                    if(!servo_group_params.second["simulation"].hasMember(init_value_param_name))
                      {
                        ROS_ERROR("can not find '%s' gazebo paramter for servo %s", init_value_param_name.c_str(),  string(servo_params.second["name"]).c_str());
                        return false;
                      }
                    // use param of servo group
                    init_value = static_cast<double>(servo_group_params.second["simulation"][init_value_param_name]);
                  }
                else
                  {
                    // use param of servo
                    init_value = static_cast<double>(servo_params.second["simulation"][init_value_param_name]);
                  }
                control_input_.at(required_id(mjOBJ_ACTUATOR, servo_name)) = init_value;
              }
          }
      }

    /* Initialize spinal interface */
    spinal_interface_.init(model_nh_, rotor_list_.size());
    registerInterface(&spinal_interface_);

    ros::NodeHandle simulation_nh = ros::NodeHandle(model_nh, "simulation");
    simulation_nh.param("ground_truth_pub_rate", ground_truth_pub_rate_, 0.01); // [sec]
    simulation_nh.param("ground_truth_pos_noise", ground_truth_pos_noise_, 0.0); // m
    simulation_nh.param("ground_truth_vel_noise", ground_truth_vel_noise_, 0.0); // m/s
    simulation_nh.param("ground_truth_rot_noise", ground_truth_rot_noise_, 0.0); // rad
    simulation_nh.param("ground_truth_angular_noise", ground_truth_angular_noise_, 0.0); // rad/s
    simulation_nh.param("ground_truth_rot_drift", ground_truth_rot_drift_, 0.0); // rad
    simulation_nh.param("ground_truth_vel_drift", ground_truth_vel_drift_, 0.0); // m/s
    simulation_nh.param("ground_truth_angular_drift", ground_truth_angular_drift_, 0.0); // rad/s
    simulation_nh.param("ground_truth_rot_drift_frequency", ground_truth_rot_drift_frequency_, 0.0); // 1/s
    simulation_nh.param("ground_truth_vel_drift_frequency", ground_truth_vel_drift_frequency_, 0.0); // 1/s
    simulation_nh.param("ground_truth_angular_drift_frequency", ground_truth_angular_drift_frequency_, 0.0); // 1/s

    simulation_nh.param("mocap_pub_rate", mocap_pub_rate_, 0.01); // [sec]
    simulation_nh.param("mocap_pos_noise", mocap_pos_noise_, 0.001); // m
    simulation_nh.param("mocap_rot_noise", mocap_rot_noise_, 0.001); // rad
    ground_truth_pub_ = model_nh.advertise<nav_msgs::Odometry>("ground_truth", 1);
    mocap_pub_ = model_nh.advertise<geometry_msgs::PoseStamped>("mocap/pose", 1);

    return true;
  }

  void AerialRobotHWSim::read(const ros::Time& time, const ros::Duration& period)
  {
    const int fc_id = fc_id_;
    mjtNum* site_xpos = mujoco_data_->site_xpos;
    mjtNum* site_xmat = mujoco_data_->site_xmat;
    tf::Matrix3x3 fc_rot_mat = tf::Matrix3x3(site_xmat[9 * fc_id + 0], site_xmat[9 * fc_id + 1], site_xmat[9 * fc_id + 2],
                                             site_xmat[9 * fc_id + 3], site_xmat[9 * fc_id + 4], site_xmat[9 * fc_id + 5],
                                             site_xmat[9 * fc_id + 6], site_xmat[9 * fc_id + 7], site_xmat[9 * fc_id + 8]);
    tf::Quaternion fc_quat;
    fc_rot_mat.getRotation(fc_quat);

    tf::Vector3 values[3];
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j)
        values[i][j] = mujoco_data_->sensordata[mujoco_model_->sensor_adr[sensor_ids_[i]] + j];
    const auto& acc = values[0];
    const auto& gyro = values[1];
    const auto& mag = values[2];

    spinal_interface_.setImuValue(acc.x(), acc.y(), acc.z(), gyro.x(), gyro.y(), gyro.z());
    spinal_interface_.setMagValue(mag.x(), mag.y(), mag.z());

    spinal_interface_.stateEstimate();

    /* publish ground truth value */
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = time;
    odom_msg.pose.pose.position.x =site_xpos[3 * fc_id + 0];
    odom_msg.pose.pose.position.y =site_xpos[3 * fc_id + 1];
    odom_msg.pose.pose.position.z =site_xpos[3 * fc_id + 2];
    odom_msg.pose.pose.orientation.x = fc_quat.x();
    odom_msg.pose.pose.orientation.y = fc_quat.y();
    odom_msg.pose.pose.orientation.z = fc_quat.z();
    odom_msg.pose.pose.orientation.w = fc_quat.w();

    /* linear velocity of the baselink (fc) in the world frame */
    mjtNum fc_vel[6];
    mj_objectVelocity(mujoco_model_, mujoco_data_, mjtObj_::mjOBJ_SITE, fc_id, fc_vel, 0);
    odom_msg.twist.twist.linear.x = fc_vel[3] + gazebo::gaussianKernel(ground_truth_vel_noise_);
    odom_msg.twist.twist.linear.y = fc_vel[4] + gazebo::gaussianKernel(ground_truth_vel_noise_);
    odom_msg.twist.twist.linear.z = fc_vel[5] + gazebo::gaussianKernel(ground_truth_vel_noise_);

    /* CAUTION! the angular is described in the fc frame, same as the gazebo simulation */
    odom_msg.twist.twist.angular.x = gyro.x() + gazebo::gaussianKernel(ground_truth_angular_noise_);
    odom_msg.twist.twist.angular.y = gyro.y() + gazebo::gaussianKernel(ground_truth_angular_noise_);
    odom_msg.twist.twist.angular.z = gyro.z() + gazebo::gaussianKernel(ground_truth_angular_noise_);

    if((time - last_ground_truth_time_).toSec() >= ground_truth_pub_rate_)
      {
        ground_truth_pub_.publish(odom_msg);
        last_ground_truth_time_ = time;
      }

    /* set ground truth for controller: use the value with noise */
    spinal_interface_.setGroundTruthStates(fc_quat.x(), fc_quat.y(), fc_quat.z(), fc_quat.w(),
                                           odom_msg.twist.twist.angular.x,
                                           odom_msg.twist.twist.angular.y,
                                           odom_msg.twist.twist.angular.z);

    if((time - last_mocap_time_).toSec() >= mocap_pub_rate_)
      {
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header.stamp = time;
        pose_msg.pose.position.x = site_xpos[3 * fc_id + 0] + gazebo::gaussianKernel(mocap_pos_noise_);
        pose_msg.pose.position.y = site_xpos[3 * fc_id + 1] + gazebo::gaussianKernel(mocap_pos_noise_);
        pose_msg.pose.position.z = site_xpos[3 * fc_id + 2] + gazebo::gaussianKernel(mocap_pos_noise_);


        tf::Quaternion q_delta;
        q_delta.setRPY(gazebo::gaussianKernel(mocap_rot_noise_),
                       gazebo::gaussianKernel(mocap_rot_noise_),
                       gazebo::gaussianKernel(mocap_rot_noise_));
        tf::Quaternion q_noise = fc_quat * q_delta;
        pose_msg.pose.orientation.x = q_noise.x();
        pose_msg.pose.orientation.y = q_noise.y();
        pose_msg.pose.orientation.z = q_noise.z();
        pose_msg.pose.orientation.w = q_noise.w();

        mocap_pub_.publish(pose_msg);
        last_mocap_time_ = time;
      }

    if ((time - last_joint_state_time_).toSec() >= joint_state_pub_rate_)
      {
        sensor_msgs::JointState msg;
        msg.header.stamp = time;
        msg.name = joint_list_;
        for (int id : joint_ids_)
          {
            msg.position.push_back(mujoco_data_->qpos[mujoco_model_->jnt_qposadr[id]]);
            msg.velocity.push_back(mujoco_data_->qvel[mujoco_model_->jnt_dofadr[id]]);
          }
        joint_state_pub_.publish(msg);
        last_joint_state_time_ = time;
      }
  }

  void AerialRobotHWSim::write(const ros::Time& time, const ros::Duration& period)
  {
    std::lock_guard<std::mutex> lock(control_mutex_);
    for(int i = 0; i < spinal_interface_.getMotorNum(); i++)
      {
        int rotor_id = mj_name2id(mujoco_model_, mjOBJ_ACTUATOR, rotor_list_.at(i).c_str());
        double rotor_force = spinal_interface_.getForce(i);
        control_input_.at(rotor_id) = rotor_force;
      }

    for (int id : actuator_ids_) mujoco_data_->ctrl[id] = control_input_[id];
  }

  void AerialRobotHWSim::controlInputCallback(const sensor_msgs::JointState& msg)
  {
    if (msg.name.size() != msg.position.size())
      {
        ROS_WARN("MuJoCo servo command names and positions must have equal sizes");
        return;
      }
    std::lock_guard<std::mutex> lock(control_mutex_);
    for (size_t i = 0; i < msg.name.size(); ++i)
      {
        const int id = mj_name2id(mujoco_model_, mjOBJ_ACTUATOR, (model_prefix_ + msg.name[i]).c_str());
        if (std::find(actuator_ids_.begin(), actuator_ids_.end(), id) == actuator_ids_.end() ||
            !std::isfinite(msg.position[i]))
          {
            ROS_WARN_STREAM("Invalid MuJoCo servo command for " << model_nh_.getNamespace() << ": " << msg.name[i]);
            continue;
          }
        control_input_[id] = msg.position[i];
      }
  }

}

PLUGINLIB_EXPORT_CLASS(mujoco_ros_control::AerialRobotHWSim, mujoco_ros_control::RobotHWSim)
