// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <aerial_robot_control/control/fully_actuated_nobend_controller.h>

using namespace std;

namespace aerial_robot_control
{
  FullyActuatedNobendController::FullyActuatedNobendController():
    PoseLinearController(),
    wrench_allocation_matrix_pub_stamp_(0),
    torque_allocation_matrix_inv_pub_stamp_(0)
  {
  }

  void FullyActuatedNobendController::initialize(ros::NodeHandle nh,
                                                   ros::NodeHandle nhp,
                                                   boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                                   boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                                   boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                                   double ctrl_loop_rate)
  {
    PoseLinearController::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_rate);

    rosParamInit();
    q_mat_.resize(6, motor_num_);
    q_mat_inv_.resize(motor_num_, 6);
    target_base_thrust_.resize(motor_num_);

    pid_msg_.x.total.resize(motor_num_),
    pid_msg_.y.total.resize(motor_num_),
    pid_msg_.z.total.resize(motor_num_),

    rpy_gain_pub_ = nh_.advertise<spinal::RollPitchYawTerms>("rpy/gain", 1);
    flight_cmd_pub_ = nh_.advertise<spinal::FourAxisCommand>("four_axes/command", 1);
    new_CoG_pub_ = nh_.advertise<geometry_msgs::Point>("newCoG", 1);
    robot_id_pub_ = nh_.advertise<std_msgs::UInt32>("robot_id", 1);
    torque_allocation_matrix_inv_pub_ = nh_.advertise<spinal::TorqueAllocationMatrixInv>("torque_allocation_matrix_inv", 1);
    wrench_allocation_matrix_pub_ = nh_.advertise<aerial_robot_msgs::WrenchAllocationMatrix>("debug/wrench_allocation_matrix", 1);
    wrench_allocation_matrix_inv_pub_ = nh_.advertise<aerial_robot_msgs::WrenchAllocationMatrix>("debug/wrench_allocation_matrix_inv", 1);

    // obtaining the original geometric configuration
    rotors_origin_original = robot_model_->getRotorsOriginFromCog<Eigen::Vector3d>();
    rotors_normal_original = robot_model_->getRotorsNormalFromCog<Eigen::Vector3d>();
    q_mat_original = Eigen::Matrix<double, 8, 8>::Zero();
    first = true; //obtaining the original WrenchMatrixOnCoG here causes it to be zero
    Att1 = Eigen::Quaterniond(1,0,0,0);
    Att2 = Eigen::Quaterniond(1,0,0,0);
    q_mat_new = Eigen::Matrix<double, 8, 8>::Zero();

    // new CoG location related
    CoG.x=0;
    CoG.y=0;
    CoG.z=0;
    Pos1Subscriber = nh_.subscribe("/assemble_quadrotors1/mocap/pose", 1, &FullyActuatedNobendController::pos1Callback, this);
    Pos2Subscriber = nh_.subscribe("/assemble_quadrotors2/mocap/pose", 1, &FullyActuatedNobendController::pos2Callback, this);
    // extracting the robot_id from robot_ns
    std::smatch match;
    robot_ns = nh_.getNamespace();
    std::regex_search(robot_ns, match, std::regex("\\d"));
    robot_id = match.empty() ? 1 : std::stoi(match[0]);
    std_msgs::UInt32 msg;
    msg.data=robot_id;
    robot_id_pub_.publish(msg);

    TargetPitch1Subscriber = nh_.subscribe("/target_pitch1",1,&FullyActuatedNobendController::TargetPitch1Callback, this);
    TargetYaw1Subscriber   = nh_.subscribe("/target_yaw1",1,&FullyActuatedNobendController::TargetYaw1Callback, this);
    TargetPitch2Subscriber = nh_.subscribe("/target_pitch2",1,&FullyActuatedNobendController::TargetPitch2Callback, this);
    TargetYaw2Subscriber   = nh_.subscribe("/target_yaw2",1,&FullyActuatedNobendController::TargetYaw2Callback, this);

    YawPublisher = nh_.advertise<aerial_robot_msgs::FlightNav>("/assemble_quadrotors"+std::to_string(robot_id)+"/uav/nav", 1);
  }

  inline void FullyActuatedNobendController::CalculateCoG(){
    boost::lock_guard<boost::mutex> lock_pos1(pos1_mutex);
    boost::lock_guard<boost::mutex> lock_pos2(pos2_mutex);
    boost::lock_guard<boost::mutex> lock_cog(cog_mutex);
            geometry_msgs::Point cog;
            cog.x = (Pos1.position.x+Pos2.position.x)/2;
            cog.y = (Pos1.position.y+Pos2.position.y)/2;
            cog.z = (Pos1.position.z+Pos2.position.z)/2;
            CoG = cog;
            new_CoG_pub_.publish(cog);
    return;
  }

  inline void FullyActuatedNobendController::CalculateRot(){
    boost::lock_guard<boost::mutex> lock_att1(att1_mutex);
    boost::lock_guard<boost::mutex> lock_att2(att2_mutex);
          Rot1 = Att1.toRotationMatrix();
          Rot2 = Att2.toRotationMatrix();
  
  
    boost::lock_guard<boost::mutex> lock_att_rel(rot_rel_mutex);
      if(robot_id==1){ //calculate {}^1 R_{2} = ({}^0 R_{1})^T {}^0 R_{2}
        Rot_rel = Rot1.transpose() * Rot2;
      }

      else{ // calculate {}^2 R_{1}
        Rot_rel = Rot2.transpose() * Rot1;
    }
    return;
  }

  inline void FullyActuatedNobendController::updateWrenchMatrixOnCoG(){
    
    boost::lock_guard<boost::mutex> lock_att_rel(rot_rel_mutex);
    boost::lock_guard<boost::mutex> lock_q(q_new_mutex);
    Eigen::Matrix3d matrix_z180;
    matrix_z180 << -1, 0, 0,
              0, -1, 0,
              0, 0, 1;
      //linear
      if(robot_id == 1){ // xyz part corresponding to quadrotor1 remains the same
        q_mat_new.block(0,0,3,4) = q_mat_original.block(0,0,3,4);
        q_mat_new.block(0,4,3,4) = Rot_rel* matrix_z180 * q_mat_original.block(0,4,3,4);
      }

      else{
        q_mat_new.block(0,4,3,4) = q_mat_original.block(0,4,3,4);
        q_mat_new.block(0,0,3,4) = Rot_rel* matrix_z180 * q_mat_original.block(0,0,3,4);
      }

      //roll
      q_mat_new.row(3)=q_mat_original.row(3);
    return;
  }
  void FullyActuatedNobendController::pos1Callback(const geometry_msgs::PoseStamped& msg){
    {
      boost::lock_guard<boost::mutex> lock_pos1(pos1_mutex);
      boost::lock_guard<boost::mutex> lock_att1(att1_mutex);
        Pos1 = msg.pose;
        geometry_msgs::Quaternion msg_quat = msg.pose.orientation;
      Att1 = Eigen::Quaterniond(msg_quat.w, msg_quat.x, msg_quat.y, msg_quat.z);
      Att1.normalize();
      }
    if(robot_id == 1){
      {CalculateCoG();}
      {CalculateRot();}
      {updateWrenchMatrixOnCoG();}
    }
    return;
  }

  void FullyActuatedNobendController::pos2Callback(const geometry_msgs::PoseStamped& msg){
    {
      boost::lock_guard<boost::mutex> lock_pos2(pos2_mutex);
      boost::lock_guard<boost::mutex> lock_att2(att2_mutex);
        Pos2 = msg.pose;
        geometry_msgs::Quaternion msg_quat = msg.pose.orientation;
      Att2 = Eigen::Quaterniond(msg_quat.w, msg_quat.x, msg_quat.y, msg_quat.z);
      Att2.normalize();
    }
    if(robot_id == 2){
      {CalculateCoG();}
      {CalculateRot();}
      {updateWrenchMatrixOnCoG();}
    }
    return;
  }

  void FullyActuatedNobendController::TargetPitch1Callback(const std_msgs::Float64& msg){
    boost::lock_guard<boost::mutex> lock(Pitch1mutex);
    ROS_INFO_STREAM("p1");
    TargetPitch1 = (msg).data;
    if(robot_id == 1){
      ROS_INFO_STREAM("set");
      navigator_->setTargetPitch(TargetPitch1);
    }
  }

  void FullyActuatedNobendController::TargetYaw1Callback(const std_msgs::Float64& msg){
    boost::lock_guard<boost::mutex> lock(Yaw1mutex);
    ROS_INFO_STREAM("y1");
    TargetYaw1 = (msg).data;
    if(robot_id == 1){
      ROS_INFO_STREAM("set");
      aerial_robot_msgs::FlightNav send_msg;
      send_msg.pos_xy_nav_mode=0;
      send_msg.yaw_nav_mode=2;
      send_msg.pos_z_nav_mode=0;
      send_msg.target_yaw=TargetYaw1;
      YawPublisher.publish(send_msg);
    }
  }

  void FullyActuatedNobendController::TargetPitch2Callback(const std_msgs::Float64& msg){
    boost::lock_guard<boost::mutex> lock(Pitch2mutex);
    ROS_INFO_STREAM("p2");
    TargetPitch2 = (msg).data;
    if(robot_id == 2){
      ROS_INFO_STREAM("set");
      navigator_->setTargetPitch(TargetPitch2);
    }
  }

  void FullyActuatedNobendController::TargetYaw2Callback(const std_msgs::Float64& msg){
    boost::lock_guard<boost::mutex> lock(Yaw2mutex);
    ROS_INFO_STREAM("y2");
    TargetYaw2 = (msg).data;
    if(robot_id == 2){
      ROS_INFO_STREAM("set");
      aerial_robot_msgs::FlightNav send_msg;
      send_msg.pos_xy_nav_mode=0;
      send_msg.yaw_nav_mode=2;
      send_msg.pos_z_nav_mode=0;
      send_msg.target_yaw=TargetYaw2;
      YawPublisher.publish(send_msg);
    }
  }

  void FullyActuatedNobendController::PIDupdate(){
    vel_ = estimator_->getVel(Frame::COG, estimate_mode_);
    target_pos_ = navigator_->getTargetPos();
    target_vel_ = navigator_->getTargetVel();
    target_acc_ = navigator_->getTargetAcc();
    geometry_msgs::Point cog;
    {boost::lock_guard<boost::mutex> lock(cog_mutex); cog = CoG; }

    rpy_ = estimator_->getEuler(Frame::COG, estimate_mode_);
    omega_ = estimator_->getAngularVel(Frame::COG, estimate_mode_);
    target_rpy_ = navigator_->getTargetRPY();
    target_omega_ = navigator_->getTargetOmega();

    // time diff
    double du = ros::Time::now().toSec() - control_timestamp_;

    // x & y
    switch(navigator_->getXyControlMode())
      {
      case aerial_robot_navigation::POS_CONTROL_MODE:
        pid_controllers_.at(X).update(target_pos_.x() - cog.x, du, target_vel_.x() - vel_.x(), target_acc_.x());
        pid_controllers_.at(Y).update(target_pos_.y() - cog.y, du, target_vel_.y() - vel_.y(), target_acc_.y());
        break;
      case aerial_robot_navigation::VEL_CONTROL_MODE:
        pid_controllers_.at(X).update(0, du, target_vel_.x() - vel_.x(), target_acc_.x());
        pid_controllers_.at(Y).update(0, du, target_vel_.y() - vel_.y(), target_acc_.y());
        break;
      case aerial_robot_navigation::ACC_CONTROL_MODE:
        pid_controllers_.at(X).update(0, du, 0, target_acc_.x());
        pid_controllers_.at(Y).update(0, du, 0, target_acc_.y());
        break;
      default:
        break;
      }

    if(navigator_->getForceLandingFlag())
      {
        pid_controllers_.at(X).reset();
        pid_controllers_.at(Y).reset();
      }

    // z
    double err_z = target_pos_.z() - cog.z;
    double err_v_z = target_vel_.z() - vel_.z();
    double du_z = du;
    double z_p_limit = pid_controllers_.at(Z).getLimitP();
    bool final_landing_phase = false;
    if(navigator_->getNaviState() == aerial_robot_navigation::LAND_STATE)
      {
        if(-err_z > safe_landing_height_)
          {
            err_z = landing_err_z_;  // too high, slowly descend
            if(vel_.z() < landing_err_z_) du_z = 0;  // freeze i term when descending
          }
        else
          {
            pid_controllers_.at(Z).setLimitP(0); // no p control in final safe landing phase
            final_landing_phase = true;
          }
      }

    if(navigator_->getForceLandingFlag())
      {
        pid_controllers_.at(Z).setLimitP(0); // no p control in force landing phase
        err_z = force_landing_descending_rate_;
        err_v_z = 0;
        target_acc_.setZ(0);
      }

    pid_controllers_.at(Z).update(err_z, du_z, err_v_z, target_acc_.z());

    if(pid_controllers_.at(Z).getErrI() < 0) pid_controllers_.at(Z).setErrI(0);

    if(final_landing_phase || navigator_->getForceLandingFlag())
      {
        pid_controllers_.at(Z).setLimitP(z_p_limit); // revert z p limit
        pid_controllers_.at(Z).setErrP(0); // for derived controller which use err_p in feedback control (e.g., LQI)
      }

    // roll pitch
    double du_rp = du;
    if(!start_rp_integration_)
      {
        if(cog.z - estimator_->getLandingHeight() > start_rp_integration_height_)
          {
            start_rp_integration_ = true;
            spinal::FlightConfigCmd flight_config_cmd;
            flight_config_cmd.cmd = spinal::FlightConfigCmd::INTEGRATION_CONTROL_ON_CMD;
            navigator_->getFlightConfigPublisher().publish(flight_config_cmd);
            ROS_WARN_ONCE("start roll/pitch I control");
          }
        du_rp = 0;
      }
    pid_controllers_.at(ROLL).update(target_rpy_.x() - rpy_.x(), du_rp, target_omega_.x() - omega_.x());
    pid_controllers_.at(PITCH).update(target_rpy_.y() - rpy_.y(), du_rp, target_omega_.y() - omega_.y());

    // yaw
    double err_yaw = angles::shortest_angular_distance(rpy_.z(), target_rpy_.z());
    double err_omega_z = target_omega_.z() - omega_.z();
    if(!need_yaw_d_control_)
      {
        err_omega_z = target_omega_.z(); // part of the control in spinal
      }
    pid_controllers_.at(YAW).update(err_yaw, du, err_omega_z);

    // update
    control_timestamp_ = ros::Time::now().toSec();

    /* ros pub */
    pid_msg_.header.stamp.fromSec(estimator_->getImuLatestTimeStamp());
    pid_msg_.x.total.at(0) = pid_controllers_.at(X).result();
    pid_msg_.x.p_term.at(0) = pid_controllers_.at(X).getPTerm();
    pid_msg_.x.i_term.at(0) = pid_controllers_.at(X).getITerm();
    pid_msg_.x.d_term.at(0) = pid_controllers_.at(X).getDTerm();
    pid_msg_.x.target_p = target_pos_.x();
    pid_msg_.x.err_p = target_pos_.x() - cog.x;
    pid_msg_.x.target_d = target_vel_.x();
    pid_msg_.x.err_d = target_vel_.x() - vel_.x();

    pid_msg_.y.total.at(0) = pid_controllers_.at(Y).result();
    pid_msg_.y.p_term.at(0) = pid_controllers_.at(Y).getPTerm();
    pid_msg_.y.i_term.at(0) = pid_controllers_.at(Y).getITerm();
    pid_msg_.y.d_term.at(0) = pid_controllers_.at(Y).getDTerm();
    pid_msg_.y.target_p = target_pos_.y();
    pid_msg_.y.err_p = target_pos_.y() - cog.x;
    pid_msg_.y.target_d = target_vel_.y();
    pid_msg_.y.err_d = target_vel_.y() - vel_.y();

    pid_msg_.z.total.at(0) = pid_controllers_.at(Z).result();
    pid_msg_.z.p_term.at(0) = pid_controllers_.at(Z).getPTerm();
    pid_msg_.z.i_term.at(0) = pid_controllers_.at(Z).getITerm();
    pid_msg_.z.d_term.at(0) = pid_controllers_.at(Z).getDTerm();
    pid_msg_.z.target_p = target_pos_.z();
    pid_msg_.z.err_p = target_pos_.z() - cog.z;
    pid_msg_.z.target_d = target_vel_.z();
    pid_msg_.z.err_d = target_vel_.z() - vel_.z();

    pid_msg_.roll.total.at(0) = pid_controllers_.at(ROLL).result();
    pid_msg_.roll.p_term.at(0) = pid_controllers_.at(ROLL).getPTerm();
    pid_msg_.roll.i_term.at(0) = pid_controllers_.at(ROLL).getITerm();
    pid_msg_.roll.d_term.at(0) = pid_controllers_.at(ROLL).getDTerm();
    pid_msg_.roll.target_p = target_rpy_.x();
    pid_msg_.roll.err_p = target_rpy_.x() - rpy_.x();
    pid_msg_.roll.target_d = target_omega_.x();
    pid_msg_.roll.err_d = target_omega_.x() - omega_.x();

    pid_msg_.pitch.total.at(0) = pid_controllers_.at(PITCH).result();
    pid_msg_.pitch.p_term.at(0) = pid_controllers_.at(PITCH).getPTerm();
    pid_msg_.pitch.i_term.at(0) = pid_controllers_.at(PITCH).getITerm();
    pid_msg_.pitch.d_term.at(0) = pid_controllers_.at(PITCH).getDTerm();
    pid_msg_.pitch.target_p = target_rpy_.y();
    pid_msg_.pitch.err_p = target_rpy_.y() - rpy_.y();
    pid_msg_.pitch.target_d = target_omega_.y();
    pid_msg_.pitch.err_d = target_omega_.y() - omega_.y();

    pid_msg_.yaw.total.at(0) = pid_controllers_.at(YAW).result();
    pid_msg_.yaw.p_term.at(0) = pid_controllers_.at(YAW).getPTerm();
    pid_msg_.yaw.i_term.at(0) = pid_controllers_.at(YAW).getITerm();
    pid_msg_.yaw.d_term.at(0) = pid_controllers_.at(YAW).getDTerm();
    pid_msg_.yaw.target_p = target_rpy_.z();
    pid_msg_.yaw.err_p = err_yaw;
    pid_msg_.yaw.target_d = target_omega_.z();
    pid_msg_.yaw.err_d = target_omega_.z() - omega_.z();

    //du pub
    du_msg_.data = du;
    du_pub_.publish(du_msg_);
  }

  void FullyActuatedNobendController::reset()
  {
    PoseLinearController::reset();
    first=true;
    setAttitudeGains();
  }

  void FullyActuatedNobendController::controlCore()
  {
    PIDupdate();
    if(first){
      { 
        boost::lock_guard<boost::mutex> lock_cog(cog_mutex);
        navigator_->setTargetPosX(CoG.x);
        navigator_->setTargetPosY(CoG.y);
      }
      {
      q_mat_original = robot_model_->calcWrenchMatrixOnCoG();
        boost::lock_guard<boost::mutex> lock(q_new_mutex);
        q_mat_new = robot_model_->calcWrenchMatrixOnCoG();
      first = false;
      }
    }

    tf::Matrix3x3 uav_rot_yaw = tf::Matrix3x3(tf::createQuaternionFromRPY(0.0, 0.0, rpy_.z()));
    tf::Matrix3x3 uav_rot_pitch_yaw = tf::Matrix3x3(tf::createQuaternionFromRPY(0.0, rpy_.y(), rpy_.z()));

    tf::Matrix3x3 uav_rot = estimator_->getOrientation(Frame::COG, estimate_mode_);
    tf::Vector3 target_acc_w(pid_controllers_.at(X).result(),
                             pid_controllers_.at(Y).result(),
                             pid_controllers_.at(Z).result());
    tf::Vector3 target_acc_cog = uav_rot.inverse() * target_acc_w;
    //tf::Vector3 target_acc_cog = uav_rot_yaw.inverse() * target_acc_w; 
    // should use uav_rot_yaw in place for uav_rot for real flight

    Eigen::Matrix3d inertia_inv = robot_model_->getInertia<Eigen::Matrix3d>().inverse();
    double mass_inv =  1 / robot_model_->getMass();
    double mass =  robot_model_->getMass();
    Eigen::MatrixXd q_mat = robot_model_->calcWrenchMatrixOnCoG();
    q_mat_.topRows(3) =  mass_inv * q_mat.topRows(3) ;
    q_mat_.bottomRows(3) =  inertia_inv * q_mat.bottomRows(3);
    // Step0: Calculate the psuedo-CoG of each drone

    std::vector<Eigen::Vector3d> rotors_origin = robot_model_->getRotorsOriginFromCog<Eigen::Vector3d>();

    Eigen::Vector3d pCoG_1 = Eigen::Vector3d::Zero();
    Eigen::Vector3d pCoG_2 = Eigen::Vector3d::Zero();
    for(int i=0;i<4;i++){
      pCoG_1 = pCoG_1 + rotors_origin[i] / 4;
      pCoG_2 = pCoG_2 + rotors_origin[i+4] / 4;
    }

    //Step0.5: obtain the normal vectors of thrusts
    std::vector<Eigen::Vector3d> rotors_normal = robot_model_->getRotorsNormalFromCog<Eigen::Vector3d>();

    Eigen::VectorXd target_thrust_x_term;
    Eigen::VectorXd target_thrust_y_term;
    Eigen::VectorXd target_thrust_z_term;
    Eigen::VectorXd target_thrust_soft_term; 

    // method 1
    if(control_method == 0){
      // Step1: create new Q-matrix
      Eigen::MatrixXd Q_new_test(8,8);
      Eigen::VectorXd Q_row4 = Eigen::VectorXd::Zero(8), Q_row5 = Eigen::VectorXd::Zero(8); // yaw
      Eigen::VectorXd Q_row6 = Eigen::VectorXd::Zero(8), Q_row7 = Eigen::VectorXd::Zero(8); // pitch
      
      for(int i=0;i<4;i++){
        
        // yaw
        Q_row4(i) = (q_mat_original.row(5))(i);
        Q_row5(i+4) = (q_mat_original.row(5))(i+4);

        //pitch
        Q_row6(i) = (q_mat_original.row(4))(i);
        Q_row7(i+4) = (q_mat_original.row(4))(i+4);
      }

      {
        boost::lock_guard<boost::mutex> lock(q_new_mutex);
        Q_new_test<<q_mat_new.row(0), q_mat_new.row(1), q_mat_new.row(2), q_mat_new.row(3), Q_row4.transpose(), Q_row5.transpose(), Q_row6.transpose(),Q_row7.transpose();
      }

      // Step2: calculate SR-inverse of the new Q

      double sr_inverse_sigma = 10;
      Eigen::MatrixXd q = Q_new_test;
      Eigen::MatrixXd q_q_t = q * q.transpose();
      Eigen::MatrixXd sr_inv = q.transpose() * (q_q_t + sr_inverse_sigma* Eigen::MatrixXd::Identity(q_q_t.cols(), q_q_t.rows())).inverse();

      // removing irravent things
      sr_inv.block(4,4,4,1) = Eigen::Vector4d::Zero(4);
      sr_inv.block(0,5,4,1) = Eigen::Vector4d::Zero(4);
      sr_inv.block(4,6,4,1) = Eigen::Vector4d::Zero(4);
      sr_inv.block(0,7,4,1) = Eigen::Vector4d::Zero(4);
      q_mat_inv_ = 10*sr_inv;
      // ROS_INFO_STREAM(q);
      
      //step3: calculate thrust accounting for softness & linear acc

      tf::Vector3 gravity_world(0,
                                0,
                                -9.8);
      //gravity_world(3) = -9.8;
      tf::Vector3 gravity_cog = uav_rot_pitch_yaw.inverse() * gravity_world;
      Eigen::Vector3d gravity_CoG(gravity_cog.x(), gravity_cog.y(), gravity_cog.z() );

      Eigen::VectorXd thrust_constant = Eigen::VectorXd::Zero(8);
      // thrust_constant << 0,0,0,0,0,(mass/2)*(pCoG_1.cross(gravity_CoG)).y(),(mass/2)*(pCoG_2.cross(gravity_CoG)).y();
      thrust_constant << /*x*/0, /*y*/0, /*z*/0, /*r*/0, /*y1*/0, /*y2*/0, /*p1*/0, /*p2*/0; // PCS gen. force feed forward
      
      target_thrust_x_term = q_mat_inv_.col(X) * target_acc_cog.x();
      target_thrust_y_term = q_mat_inv_.col(Y) * target_acc_cog.y();
      target_thrust_z_term = q_mat_inv_.col(Z) * target_acc_cog.z();
      target_thrust_soft_term = q_mat_inv_*thrust_constant;
      // ROS_INFO_STREAM(target_thrust_soft_term);
    }

    // method 2
    else{
      //
      double mass_inv =  1 / robot_model_->getMass();
      Eigen::MatrixXd WrenchMatrixOnCoG = robot_model_->calcWrenchMatrixOnCoG();
      Eigen::MatrixXd q_bottom_q1(3,8), q_bottom_q2(3,8), q1_bottom(3,8), q2_bottom(3,8);

      q_bottom_q1 = WrenchMatrixOnCoG.bottomRows(3);
      q_bottom_q2 = WrenchMatrixOnCoG.bottomRows(3);
      
      Eigen::VectorXd Q_row6 = Eigen::VectorXd::Zero(8), Q_row7 = Eigen::VectorXd::Zero(8);
      
      for(int i=0;i<4;i++){
        Q_row6(i) = ( (rotors_origin[i]-pCoG_1).cross(rotors_normal[i]) ).y();
        Q_row7(i+4) = ( (rotors_origin[i+4]-pCoG_2).cross(rotors_normal[i+4]) ).y();
      }

      q_bottom_q1.row(1)=Q_row6;
      q_bottom_q2.row(1)=Q_row7;

      q1_bottom = q_bottom_q1;
      q2_bottom = q_bottom_q2;


      //ROS_INFO_STREAM(rotors_normal[0]);
      Eigen::MatrixXd Q_new_test(7,8);
      Q_new_test<<q_mat.row(0), q_mat.row(1), q_mat.row(2), q_mat.row(3) ,q_mat.row(5), q1_bottom.row(1), q2_bottom.row(1);
    
      // Step2: calculate SR-inverse of the new Q
      double sr_inverse_sigma = 0.1;
      Eigen::MatrixXd q = Q_new_test;
      Eigen::MatrixXd q_q_t = q * q.transpose();
      Eigen::MatrixXd sr_inv = q.transpose() * (q_q_t + sr_inverse_sigma* Eigen::MatrixXd::Identity(q_q_t.cols(), q_q_t.rows())).inverse();

      q_mat_inv_=sr_inv;

      //step3: calculate thrust accounting for softness & linear acc
      target_thrust_x_term = q_mat_inv_.col(X) * target_acc_cog.x();
      target_thrust_y_term = q_mat_inv_.col(Y) * target_acc_cog.y();
      target_thrust_z_term = q_mat_inv_.col(Z) * target_acc_cog.z();
      target_thrust_soft_term = Eigen::VectorXd::Zero(8).transpose(); 
    
    }

    // constraint x and y
    int index;
    double max_term = target_thrust_x_term.cwiseAbs().maxCoeff(&index);
    double residual = max_term - pid_controllers_.at(X).getLimitSum();
    if(residual > 0)
      {
        ROS_DEBUG("the position x control exceed the limit in rotor %d, %f vs %f ", index, max_term, pid_controllers_.at(X).getLimitSum());
        target_thrust_x_term *= (1 - residual / max_term);
      }

    max_term = target_thrust_y_term.cwiseAbs().maxCoeff(&index);
    residual = max_term - pid_controllers_.at(Y).getLimitSum();
    if(residual > 0)
      {
        ROS_DEBUG("the position y control exceed the limit in rotor %d, %f vs %f ", index, max_term, pid_controllers_.at(Y).getLimitSum());
        target_thrust_y_term *= (1 - residual / max_term);
      }

    // constraint z (also  I term)
    max_term = target_thrust_z_term.cwiseAbs().maxCoeff(&index);
    residual = max_term - pid_controllers_.at(Z).getLimitSum();
    if(residual > 0)
      {
        pid_controllers_.at(Z).setErrI(pid_controllers_.at(Z).getPrevErrI());
        target_thrust_z_term *= (1 - residual / max_term);
      }


    for(int i = 0; i < motor_num_; i++)
      {
        target_base_thrust_.at(i) = target_thrust_x_term(i) + target_thrust_y_term(i) + target_thrust_z_term(i) + target_thrust_soft_term(i);

        pid_msg_.x.total.at(i) =  target_thrust_x_term(i);
        pid_msg_.y.total.at(i) =  target_thrust_y_term(i);
        pid_msg_.z.total.at(i) =  target_thrust_z_term(i);
      }

    // special process for yaw since the bandwidth between PC and spinal
    double max_yaw_scale = 0; // for reconstruct yaw control term in spinal
    for (unsigned int i = 0; i < motor_num_; i++)
      {
        if(q_mat_inv_(i, YAW) > max_yaw_scale) max_yaw_scale = q_mat_inv_(i, YAW);
      }
    candidate_yaw_term_ = pid_controllers_.at(YAW).result() * max_yaw_scale;
  }

  void FullyActuatedNobendController::sendCmd()
  {
    PoseLinearController::sendCmd();

    if (ros::Time::now().toSec() - wrench_allocation_matrix_pub_stamp_ > wrench_allocation_matrix_pub_interval_)
      {
        wrench_allocation_matrix_pub_stamp_ = ros::Time::now().toSec();
        aerial_robot_msgs::WrenchAllocationMatrix msg;
        for (unsigned int i = 0; i < motor_num_; i++) {
          msg.f_x.push_back(q_mat_(0, i));
          msg.f_y.push_back(q_mat_(1, i));
          msg.f_z.push_back(q_mat_(2, i));
          msg.t_x.push_back(q_mat_(3, i));
          msg.t_y.push_back(q_mat_(4, i));
          msg.t_z.push_back(q_mat_(5, i));
        }

        wrench_allocation_matrix_pub_.publish(msg);

        for (unsigned int i = 0; i < motor_num_; i++) {
          msg.f_x.push_back(q_mat_inv_(i, 0));
          msg.f_y.push_back(q_mat_inv_(i, 1));
          msg.f_z.push_back(q_mat_inv_(i, 2));
          msg.t_x.push_back(q_mat_inv_(i, 3));
          msg.t_y.push_back(q_mat_inv_(i, 4));
          msg.t_z.push_back(q_mat_inv_(i, 5));
        }
        wrench_allocation_matrix_inv_pub_.publish(msg);
      }

    sendFourAxisCommand();

    sendTorqueAllocationMatrixInv();
  }

  void FullyActuatedNobendController::sendFourAxisCommand()
  {
    spinal::FourAxisCommand flight_command_data;
    flight_command_data.angles[0] = navigator_->getTargetRPY().x();
    flight_command_data.angles[1] = navigator_->getTargetRPY().y();
    flight_command_data.angles[2] = candidate_yaw_term_;
    flight_command_data.base_thrust = target_base_thrust_;
    flight_cmd_pub_.publish(flight_command_data);
  }

  void FullyActuatedNobendController::sendTorqueAllocationMatrixInv()
  {
    if (ros::Time::now().toSec() - torque_allocation_matrix_inv_pub_stamp_ > torque_allocation_matrix_inv_pub_interval_)
      {
        torque_allocation_matrix_inv_pub_stamp_ = ros::Time::now().toSec();

        spinal::TorqueAllocationMatrixInv torque_allocation_matrix_inv_msg;
        torque_allocation_matrix_inv_msg.rows.resize(motor_num_);
        Eigen::MatrixXd torque_allocation_matrix_inv(8,3);
        if(control_method == 0){
          torque_allocation_matrix_inv<<q_mat_inv_.col(3),(q_mat_inv_.col(5) + q_mat_inv_.col(6)),q_mat_inv_.col(4);
        }
        else{
          torque_allocation_matrix_inv<<q_mat_inv_.col(3),(q_mat_inv_.col(5)+q_mat_inv_.col(6)),q_mat_inv_.col(4);
        }
        if (torque_allocation_matrix_inv.cwiseAbs().maxCoeff() > INT16_MAX * 0.001f)
          ROS_ERROR("Torque Allocation Matrix overflow");
        for (unsigned int i = 0; i < motor_num_; i++)
          {
            torque_allocation_matrix_inv_msg.rows.at(i).x = torque_allocation_matrix_inv(i,0) * 1000;
            torque_allocation_matrix_inv_msg.rows.at(i).y = torque_allocation_matrix_inv(i,1) * 1000;
            torque_allocation_matrix_inv_msg.rows.at(i).z = torque_allocation_matrix_inv(i,2) * 1000;
          }
        torque_allocation_matrix_inv_pub_.publish(torque_allocation_matrix_inv_msg);
      }
  }

  void FullyActuatedNobendController::rosParamInit()
  {
    ros::NodeHandle control_nh(nh_, "controller");
    getParam<double>(control_nh, "torque_allocation_matrix_inv_pub_interval", torque_allocation_matrix_inv_pub_interval_, 0.05);
    getParam<double>(control_nh, "wrench_allocation_matrix_pub_interval", wrench_allocation_matrix_pub_interval_, 0.1);
    getParam<int>(control_nh, "control_method", control_method, 0);
  }

  void FullyActuatedNobendController::setAttitudeGains()
  {
    spinal::RollPitchYawTerms rpy_gain_msg; //for rosserial
    /* to flight controller via rosserial scaling by 1000 */
    rpy_gain_msg.motors.resize(1);
    rpy_gain_msg.motors.at(0).roll_p = pid_controllers_.at(ROLL).getPGain() * 1000;
    rpy_gain_msg.motors.at(0).roll_i = pid_controllers_.at(ROLL).getIGain() * 1000;
    rpy_gain_msg.motors.at(0).roll_d = pid_controllers_.at(ROLL).getDGain() * 1000;
    rpy_gain_msg.motors.at(0).pitch_p = pid_controllers_.at(PITCH).getPGain() * 1000;
    rpy_gain_msg.motors.at(0).pitch_i = pid_controllers_.at(PITCH).getIGain() * 1000;
    rpy_gain_msg.motors.at(0).pitch_d = pid_controllers_.at(PITCH).getDGain() * 1000;
    rpy_gain_msg.motors.at(0).yaw_d = pid_controllers_.at(YAW).getDGain() * 1000;
    rpy_gain_pub_.publish(rpy_gain_msg);
  }

} //namespace aerial_robot_control

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::FullyActuatedNobendController, aerial_robot_control::ControlBase);