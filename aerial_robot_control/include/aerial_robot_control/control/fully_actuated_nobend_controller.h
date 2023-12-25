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
#include <aerial_robot_control/control/pose_linear_controller.h>
#include <spinal/FourAxisCommand.h>
#include <spinal/RollPitchYawTerms.h>
#include <spinal/TorqueAllocationMatrixInv.h>
#include <spinal/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <string>
#include <regex>
#include <std_msgs/UInt32.h>
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

    // saving the original orientation of propellers
    std::vector<Eigen::Vector3d> rotors_origin_original;
    std::vector<Eigen::Vector3d> rotors_normal_original;
    Eigen::MatrixXd q_mat_original;
    bool first;

    // listeners and callback functions for updating CoG location
    ros::Subscriber Pos1Subscriber, Pos2Subscriber;
    geometry_msgs::Pose Pos1, Pos2;
    geometry_msgs::Point CoG;
    boost::mutex pos1_mutex, pos2_mutex, cog_mutex;
    inline void CalculateCoG();
    void pos1Callback(const geometry_msgs::PoseStamped& msg);
    void pos2Callback(const geometry_msgs::PoseStamped& msg);

    // Position PID for new CoG
    boost::shared_ptr<PID> pid_x, pid_y, pid_z;
    void PIDupdate();
    ros::Publisher new_CoG_pub_;

    // Attitude of the two frames & updating q_mat
    Eigen::Quaterniond Att1, Att2;
    boost::mutex att1_mutex, att2_mutex, rot_rel_mutex, q_new_mutex;
    Eigen::Matrix3d Rot1, Rot2, Rot_rel; // Rot_rel: frame {1 or 2}->{2 or 1}
    inline void CalculateRot();
    inline void updateWrenchMatrixOnCoG();
    std::string robot_ns;
    int robot_id;
    Eigen::MatrixXd q_mat_new;
    ros::Publisher robot_id_pub_;

    // PCS feed forward
    double pitch_pcs_gain, yaw_pcs_gain;
    double opposite_pitch_angle, opposite_yaw_angle; boost::mutex opposite_angle_mutex; 

    ros::Subscriber imuSubscriber;
    void imuCallback(const spinal::Imu& msg);

    double strain_pitch, strain_yaw;

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
    int control_method;
    void rosParamInit();

    // // target angle
    ros::Subscriber TargetPitch1Subscriber;
    ros::Subscriber TargetYaw1Subscriber;
    ros::Subscriber TargetPitch2Subscriber;
    ros::Subscriber TargetYaw2Subscriber;

    ros::Publisher YawPublisher;

    double TargetPitch1, TargetPitch2, TargetYaw1, TargetYaw2;
    boost::mutex Pitch1mutex, Pitch2mutex, Yaw1mutex, Yaw2mutex;

    void TargetPitch1Callback(const std_msgs::Float64& msg);
    void TargetYaw1Callback(const std_msgs::Float64& msg);
    void TargetPitch2Callback(const std_msgs::Float64& msg);
    void TargetYaw2Callback(const std_msgs::Float64& msg);

    double initial_yaw;

  };
} //namespace aerial_robot_control