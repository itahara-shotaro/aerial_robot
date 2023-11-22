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
#pragma once

#include <aerial_robot_msgs/WrenchAllocationMatrix.h>
#include <aerial_robot_control/control/base/pose_linear_controller.h>
#include <spinal/FourAxisCommand.h>
#include <spinal/RollPitchYawTerms.h>
#include <spinal/TorqueAllocationMatrixInv.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <aerial_robot_msgs/FlightNav.h>

using boost::algorithm::clamp;

namespace aerial_robot_control
{
  class FullyActuatedNobendController: public PoseLinearController
  {
  public:
    FullyActuatedNobendController();
    virtual ~FullyActuatedNobendController() = default;

    void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                    boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                    double ctrl_loop_rate) override;

    virtual void reset() override;

    virtual void controlCore() override;
    virtual void sendCmd() override;

    std::vector<float> getTargetBaseThrust() { return target_base_thrust_; }
    double getCandidateYawTerm() { return candidate_yaw_term_; }
    Eigen::MatrixXd getQMatInv() const { return q_mat_inv_;}

  private:

    Eigen::MatrixXd q_mat_;
    Eigen::MatrixXd q_mat_inv_;

    std::vector<float> target_base_thrust_;
    double candidate_yaw_term_;

    void setAttitudeGains();

    void sendFourAxisCommand();
    void sendTorqueAllocationMatrixInv();

    tf::TransformListener listener; // listening to transform of robot2
    tf::StampedTransform transform; // transform of robot2
    
    // PID for frame 2
    boost::shared_ptr<PID> pid_pitch_2;
    boost::shared_ptr<PID> pid_yaw_2;

    double pitch_2_p_gain, pitch_2_i_gain, pitch_2_d_gain, pitch_2_limit_sum, pitch_2_limit_p, pitch_2_limit_i, pitch_2_limit_d, pitch_2_limit_err_p, pitch_2_limit_err_i, pitch_2_limit_err_d;
    double yaw_2_p_gain, yaw_2_i_gain, yaw_2_d_gain, yaw_2_limit_sum, yaw_2_limit_p, yaw_2_limit_i, yaw_2_limit_d, yaw_2_limit_err_p, yaw_2_limit_err_i, yaw_2_limit_err_d;

    // getting twist of frame 2 from the python script
    ros::Subscriber twistSubscriber;
    void twistCallback(const geometry_msgs::Twist::ConstPtr& msg);
    boost::mutex twistmutex;
    geometry_msgs::Twist latestTwist;

    // retaining the original geometrical configuration
    std::vector<Eigen::Vector3d> rotors_origin_original;
    std::vector<Eigen::Vector3d> rotors_normal_original;
    Eigen::MatrixXd q_mat_original;
    bool first;

    // receiving the target angle
    ros::Subscriber TargetPitch1Subscriber;
    ros::Subscriber TargetYaw1Subscriber;
    ros::Subscriber TargetPitch2Subscriber;
    ros::Subscriber TargetYaw2Subscriber;

    double TargetPitch1, TargetYaw1, TargetPitch2, TargetYaw2;
    boost::mutex Pitch1mutex, Yaw1mutex, Pitch2mutex, Yaw2mutex;

    void TargetPitch1Callback(const std_msgs::Float64& msg);
    void TargetYaw1Callback(const std_msgs::Float64& msg);
    void TargetPitch2Callback(const std_msgs::Float64& msg);
    void TargetYaw2Callback(const std_msgs::Float64& msg);

    // inner force compensation gain
    double pitch_pcs_gain, yaw_pcs_gain;

    //void BaseNavigator::naviCallback(const aerial_robot_msgs::FlightNavConstPtr & msg)

  protected:
    ros::Publisher rpy_gain_pub_; //for spinal
    ros::Publisher torque_allocation_matrix_inv_pub_; //for spinal
    double torque_allocation_matrix_inv_pub_stamp_;
    ros::Publisher wrench_allocation_matrix_pub_; //for debug
    ros::Publisher wrench_allocation_matrix_inv_pub_; //for debug
    double wrench_allocation_matrix_pub_stamp_;

    ros::Publisher flight_cmd_pub_; //for spinal
    double torque_allocation_matrix_inv_pub_interval_;
    double wrench_allocation_matrix_pub_interval_;
    void rosParamInit();

    int control_method;


  };
} //namespace aerial_robot_control
