// -*- mode: c++ -*-

#pragma once

#include <aerial_robot_control/control/under_actuated_lqi_controller.h>
#include <aerial_robot_estimation/state_estimation.h>
#include <gimbalrotor/model/gimbalrotor_robot_model.h>

#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/UInt8.h>

namespace aerial_robot_control
{
  /* LQI controller for the gimbalrotor.

     Unlike the default GimbalrotorController (PID + pseudo-inverse allocation), the LQ problem is
     solved over the *virtual inputs* that the gimbal-masked allocation defines: (gimbal_dof + 1)
     inputs per rotor, i.e. 2N for the usual 1-DoF gimbal. The resulting per-input P/I/D gains are
     published on rpy/gain, so spinal's existing gimbal_dof path runs z/roll/pitch/yaw at 1 kHz and
     derives the gimbal angles onboard. x/y remain a PC-side outer loop, allocated in the null space
     of z/roll/pitch/yaw so it cannot disturb the fast loop.

     A commanded body rotation (final_target_baselink_rot / _rpy) is handled by the navigator as a
     redefinition of the CoG frame, not as an attitude setpoint, so the attitude target sent to
     spinal stays level here. */
  class GimbalLQIController: public UnderActuatedLQIController
  {
  public:
    GimbalLQIController();
    ~GimbalLQIController() = default;

    void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                    boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                    double ctrl_loop_rate) override;

    bool update() override;

  private:
    ros::Publisher gimbal_dof_pub_;
    ros::Publisher target_vectoring_force_pub_;

    boost::shared_ptr<GimbalrotorRobotModel> gimbalrotor_robot_model_;

    int gimbal_dof_;
    int rotor_coef_;   // virtual inputs per rotor
    int input_num_;    // motor_num_ * rotor_coef_

    double r_thrust_, r_lateral_;
    double allocation_det_thresh_;
    double max_gimbal_angle_;

    /* guards the gain-generator thread, which the base class starts before this class has finished
       resizing the gain containers to input_num_ */
    bool lqi_ready_;

    Eigen::MatrixXd q_mat_inv_;      // pseudo-inverse of the 4 x input_num_ controlled plant
    Eigen::VectorXd target_vectoring_f_;

    void rosParamInit() override;
    void controlCore() override;
    void allocateYawTerm() override;
    void sendCmd() override;
    bool optimalGain() override;
    void clampGain() override;
    void publishGain() override;
    bool checkRobotModel() override;
  };
};

