#pragma once

#include <aerial_robot_estimation/state_estimation.h>
#include <gimbalrotor/model/gimballqi_robot_model.h>
#include <aerial_robot_control/control/under_actuated_tilted_lqi_controller.h>

#include <numeric>
#include <thread>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/UInt32.h>
#include <gazebo_msgs/ApplyBodyWrench.h>
#include <gazebo_msgs/BodyRequest.h>
#include <ros/ros.h>
#include <spinal/RollPitchYawTerm.h>
#include <spinal/PMatrixPseudoInverseWithInertia.h>

namespace aerial_robot_control
{
  class GimbalLQIController: public UnderActuatedTiltedLQIController
  {
  public:
    GimbalLQIController();
    ~GimbalLQIController() = default;

    void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                    boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                    double ctrl_loop_rate) override;

  private:
    ros::Publisher gimbal_control_pub_;
    ros::Publisher gimbal_target_force_pub_;
    ros::Subscriber att_control_feedback_state_sub_;

    void gimbalControl();
    void controlCore() override;
    void rosParamInit() override;
    void sendCmd() override;
    void allocateYawTerm() override {} // do nothing

    boost::shared_ptr<GimbalLQIRobotModel> gimbalrotor_robot_model_;
    Eigen::MatrixXd P_xy_;

    bool gimbal_vectoring_check_flag_;
    bool add_lqi_result_;
    std::vector<double> lqi_att_terms_;
    std::vector<double> target_gimbal_angles_;

    double gimbal_roll_pitch_control_rate_thresh_;
    double gimbal_roll_pitch_control_p_det_thresh_;

  };
};
