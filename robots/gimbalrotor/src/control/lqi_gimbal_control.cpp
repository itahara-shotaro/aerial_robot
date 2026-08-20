#include <gimbalrotor/control/lqi_gimbal_control.h>

using namespace aerial_robot_control;

GimbalLQIController::GimbalLQIController():
  UnderActuatedLQIController(),
  gimbal_dof_(1), rotor_coef_(2), input_num_(0),
  r_thrust_(1.0), r_lateral_(10.0), allocation_det_thresh_(1e-3), max_gimbal_angle_(0.6),
  lqi_ready_(false)
{
}

void GimbalLQIController::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                     boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                     boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                     boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                     double ctrl_loop_rate)
{
  /* this also runs rosParamInit(), which sets gimbal_dof_ / rotor_coef_ / input_num_ */
  UnderActuatedLQIController::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_rate);

  gimbalrotor_robot_model_ = boost::dynamic_pointer_cast<GimbalrotorRobotModel>(robot_model);
  if(gimbalrotor_robot_model_ == nullptr)
    {
      ROS_FATAL_NAMED("LQI gimbal control", "the robot model plugin is not gimbalrotor_robot_model");
      return;
    }

  /* the base sized everything for motor_num_ thrusts; this controller works on input_num_ virtual
     inputs instead, so widen every per-input container */
  roll_gains_.resize(input_num_, Eigen::Vector3d::Zero());
  pitch_gains_.resize(input_num_, Eigen::Vector3d::Zero());
  yaw_gains_.resize(input_num_, Eigen::Vector3d::Zero());
  z_gains_.resize(input_num_, Eigen::Vector3d::Zero());

  target_base_thrust_.resize(input_num_, 0);
  pid_msg_.z.total.resize(input_num_);
  pid_msg_.z.p_term.resize(input_num_);
  pid_msg_.z.i_term.resize(input_num_);
  pid_msg_.z.d_term.resize(input_num_);
  pid_msg_.yaw.total.resize(input_num_);
  pid_msg_.yaw.p_term.resize(input_num_);
  pid_msg_.yaw.i_term.resize(input_num_);
  pid_msg_.yaw.d_term.resize(input_num_);

  target_vectoring_f_ = Eigen::VectorXd::Zero(input_num_);
  q_mat_inv_ = Eigen::MatrixXd::Zero(input_num_, 4);

  gimbal_dof_pub_ = nh_.advertise<std_msgs::UInt8>("gimbal_dof", 1);
  target_vectoring_force_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("debug/target_vectoring_force", 1);

  lqi_ready_ = true;
}

void GimbalLQIController::rosParamInit()
{
  UnderActuatedLQIController::rosParamInit();

  ros::NodeHandle control_nh(nh_, "controller");
  ros::NodeHandle lqi_nh(control_nh, "lqi");

  getParam<int>(control_nh, "gimbal_dof", gimbal_dof_, 1);
  rotor_coef_ = gimbal_dof_ + 1;
  input_num_ = motor_num_ * rotor_coef_;

  bool gimbal_calc_in_fc;
  getParam<bool>(control_nh, "gimbal_calc_in_fc", gimbal_calc_in_fc, true);
  if(!gimbal_calc_in_fc)
    ROS_ERROR_NAMED("LQI gimbal control",
                    "gimbal_calc_in_fc is false, but this controller always lets the flight controller "
                    "compute the gimbal angles: the PC never sees spinal's roll/pitch thrust term and "
                    "so cannot resolve the total force direction. Ignoring the parameter.");

  /* the plant always covers z, roll, pitch and yaw: the gimbals give real yaw authority, so there is
     no reason to fall back to the 3-axis mode that fixed-rotor platforms use */
  if(lqi_mode_ != 4)
    {
      ROS_WARN_NAMED("LQI gimbal control", "lqi_mode is forced to 4 for the gimbalrotor");
      lqi_mode_ = 4;
    }

  if(gyro_moment_compensation_)
    {
      ROS_WARN_NAMED("LQI gimbal control",
                     "gyro_moment_compensation is not supported: the inherited P matrix is sized for "
                     "thrusts, not for the masked virtual inputs. Disabling it.");
      gyro_moment_compensation_ = false;
    }

  getParam<double>(lqi_nh, "allocation_det_thresh", allocation_det_thresh_, 1e-3);
  getParam<double>(lqi_nh, "r_thrust", r_thrust_, 1.0);
  getParam<double>(lqi_nh, "r_lateral", r_lateral_, 10.0);
  getParam<double>(control_nh, "max_gimbal_angle", max_gimbal_angle_, 0.6);

  /* R, one entry per virtual input. Within each rotor's block the last component is the thrust
     direction (arm z, rotor-driven and fast); the preceding ones are the lateral directions the
     gimbal servo has to produce, and are penalised harder so the LQ solution does not put attitude
     bandwidth on the servos. */
  r_.assign(input_num_, r_thrust_);
  for(int i = 0; i < motor_num_; i++)
    for(int j = 0; j < rotor_coef_ - 1; j++)
      r_.at(rotor_coef_ * i + j) = r_lateral_;

  /* explicit per-input override, e.g. for an asymmetric airframe */
  std::vector<double> r_override;
  if(lqi_nh.getParam("r", r_override))
    {
      if((int)r_override.size() == input_num_) r_ = r_override;
      else ROS_WARN_NAMED("LQI gimbal control", "controller/lqi/r has %d entries, expected %d. Ignoring it.",
                          (int)r_override.size(), input_num_);
    }
}

bool GimbalLQIController::checkRobotModel()
{
  if(!lqi_ready_) return false;

  if(!robot_model_->initialized())
    {
      ROS_DEBUG_NAMED("LQI gimbal control", "robot model is not initialized");
      return false;
    }

  Eigen::MatrixXd alloc = gimbalrotor_robot_model_->calcMaskedWrenchMatrixOnCoG(gimbal_dof_);
  double det = (alloc * alloc.transpose()).determinant();
  if(det < allocation_det_thresh_)
    {
      ROS_WARN_THROTTLE_NAMED(1.0, "LQI gimbal control", "degenerate allocation, det: %f", det);
      return false;
    }

  return true;
}

bool GimbalLQIController::optimalGain()
{
  Eigen::MatrixXd alloc = gimbalrotor_robot_model_->calcMaskedWrenchMatrixOnCoG(gimbal_dof_);

  /* controlled axes: z, roll, pitch, yaw (already in acceleration units) */
  Eigen::MatrixXd P_dash = Eigen::MatrixXd::Zero(4, input_num_);
  P_dash.row(0) = alloc.row(2);
  P_dash.bottomRows(3) = alloc.bottomRows(3);

  /* 12 states: [z, z_dot, r, r_dot, p, p_dot, y, y_dot, |z, |r, |p, |y] */
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(12, 12);
  Eigen::MatrixXd B = Eigen::MatrixXd::Zero(12, input_num_);
  Eigen::MatrixXd C = Eigen::MatrixXd::Zero(4, 12);
  for(int i = 0; i < 4; i++)
    {
      A(2 * i, 2 * i + 1) = 1;
      B.row(2 * i + 1) = P_dash.row(i);
      C(i, 2 * i) = 1;
    }
  A.block(8, 0, 4, 12) = -C;

  Eigen::VectorXd q_diagonals(12);
  q_diagonals << lqi_z_weight_(0), lqi_z_weight_(2),
    lqi_roll_pitch_weight_(0), lqi_roll_pitch_weight_(2),
    lqi_roll_pitch_weight_(0), lqi_roll_pitch_weight_(2),
    lqi_yaw_weight_(0), lqi_yaw_weight_(2),
    lqi_z_weight_(1), lqi_roll_pitch_weight_(1), lqi_roll_pitch_weight_(1), lqi_yaw_weight_(1);
  Eigen::MatrixXd Q = q_diagonals.asDiagonal();

  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(input_num_, input_num_);
  for(int i = 0; i < input_num_; ++i) R(i, i) = r_.at(i);

  if(K_.rows() != input_num_ || K_.cols() != 12) resetGain();

  bool use_kleinman_method = !(K_.cols() == 0 || K_.rows() == 0);
  if(!control_utils::care(A, B, R, Q, K_, use_kleinman_method))
    {
      ROS_ERROR_STREAM_NAMED("LQI gimbal control", "error in solver of continuous-time algebraic riccati equation");
      return false;
    }

  for(int i = 0; i < input_num_; ++i)
    {
      z_gains_.at(i)     = Eigen::Vector3d(-K_(i, 0), K_(i,  8), -K_(i, 1));
      roll_gains_.at(i)  = Eigen::Vector3d(-K_(i, 2), K_(i,  9), -K_(i, 3));
      pitch_gains_.at(i) = Eigen::Vector3d(-K_(i, 4), K_(i, 10), -K_(i, 5));
      yaw_gains_.at(i)   = Eigen::Vector3d(-K_(i, 6), K_(i, 11), -K_(i, 7));
    }

  return true;
}

void GimbalLQIController::clampGain()
{
  /* avoid the violation of int16_t range because of spinal::RollPitchYawTerms */
  const double max_gain_thresh = 32.767;
  double max_roll_p_gain = 0, max_roll_d_gain = 0, max_pitch_p_gain = 0, max_pitch_d_gain = 0, max_yaw_d_gain = 0;
  for(int i = 0; i < input_num_; ++i)
    {
      max_roll_p_gain = std::max(max_roll_p_gain, fabs(roll_gains_.at(i)[0]));
      max_roll_d_gain = std::max(max_roll_d_gain, fabs(roll_gains_.at(i)[2]));
      max_pitch_p_gain = std::max(max_pitch_p_gain, fabs(pitch_gains_.at(i)[0]));
      max_pitch_d_gain = std::max(max_pitch_d_gain, fabs(pitch_gains_.at(i)[2]));
      max_yaw_d_gain = std::max(max_yaw_d_gain, fabs(yaw_gains_.at(i)[2]));
    }

  auto scale = [&max_gain_thresh](double max_gain, const char* name)
    {
      if(max_gain <= max_gain_thresh) return 1.0;
      ROS_WARN_STREAM_NAMED("LQI gimbal control", "the max " << name << " gain violates the range of int16_t: " << max_gain);
      return max_gain_thresh / max_gain;
    };

  double roll_p_scale = scale(max_roll_p_gain, "roll p");
  double roll_d_scale = scale(max_roll_d_gain, "roll d");
  double pitch_p_scale = scale(max_pitch_p_gain, "pitch p");
  double pitch_d_scale = scale(max_pitch_d_gain, "pitch d");
  double yaw_d_scale = scale(max_yaw_d_gain, "yaw d");

  for(int i = 0; i < input_num_; ++i)
    {
      roll_gains_.at(i)[0] *= roll_p_scale;
      roll_gains_.at(i)[2] *= roll_d_scale;
      pitch_gains_.at(i)[0] *= pitch_p_scale;
      pitch_gains_.at(i)[2] *= pitch_d_scale;
      yaw_gains_.at(i)[2] *= yaw_d_scale;
    }
}

void GimbalLQIController::publishGain()
{
  /* Spinal resolves each gimbal angle from the total commanded force, atan2(-f_lateral, f_thrust).
     Between arming and takeoff the controller does not publish a FourAxisCommand yet, so spinal's
     base thrust is still zero; if it already held the attitude gains, the attitude term alone would
     decide that angle and the gimbals would thrash on noise. Hold the gains back until the robot is
     actually flying, which is also what the PID controller effectively does (its gains only take
     effect once torque_allocation_matrix_inv arrives at takeoff). */
  int navi_state = navigator_->getNaviState();
  if(navi_state != aerial_robot_navigation::TAKEOFF_STATE &&
     navi_state != aerial_robot_navigation::HOVER_STATE &&
     navi_state != aerial_robot_navigation::LAND_STATE)
    return;

  aerial_robot_msgs::FourAxisGain four_axis_gain_msg;
  spinal::RollPitchYawTerms rpy_gain_msg; // to spinal
  rpy_gain_msg.motors.resize(input_num_);

  for(int i = 0; i < input_num_; ++i)
    {
      four_axis_gain_msg.roll_p_gain.push_back(roll_gains_.at(i)[0]);
      four_axis_gain_msg.roll_i_gain.push_back(roll_gains_.at(i)[1]);
      four_axis_gain_msg.roll_d_gain.push_back(roll_gains_.at(i)[2]);

      four_axis_gain_msg.pitch_p_gain.push_back(pitch_gains_.at(i)[0]);
      four_axis_gain_msg.pitch_i_gain.push_back(pitch_gains_.at(i)[1]);
      four_axis_gain_msg.pitch_d_gain.push_back(pitch_gains_.at(i)[2]);

      four_axis_gain_msg.yaw_p_gain.push_back(yaw_gains_.at(i)[0]);
      four_axis_gain_msg.yaw_i_gain.push_back(yaw_gains_.at(i)[1]);
      four_axis_gain_msg.yaw_d_gain.push_back(yaw_gains_.at(i)[2]);

      four_axis_gain_msg.z_p_gain.push_back(z_gains_.at(i)[0]);
      four_axis_gain_msg.z_i_gain.push_back(z_gains_.at(i)[1]);
      four_axis_gain_msg.z_d_gain.push_back(z_gains_.at(i)[2]);

      /* to flight controller via rosserial, scaled by 1000 */
      rpy_gain_msg.motors[i].roll_p = roll_gains_.at(i)[0] * 1000;
      rpy_gain_msg.motors[i].roll_i = roll_gains_.at(i)[1] * 1000;
      rpy_gain_msg.motors[i].roll_d = roll_gains_.at(i)[2] * 1000;

      rpy_gain_msg.motors[i].pitch_p = pitch_gains_.at(i)[0] * 1000;
      rpy_gain_msg.motors[i].pitch_i = pitch_gains_.at(i)[1] * 1000;
      rpy_gain_msg.motors[i].pitch_d = pitch_gains_.at(i)[2] * 1000;

      rpy_gain_msg.motors[i].yaw_d = yaw_gains_.at(i)[2] * 1000;
    }

  rpy_gain_pub_.publish(rpy_gain_msg);
  four_axis_gain_pub_.publish(four_axis_gain_msg);
}

void GimbalLQIController::controlCore()
{
  PoseLinearController::controlCore();

  /* A commanded body rotation arrives as a CoG frame redefinition (the navigator calls
     setCogDesireOrientation() and sends spinal a DesireCoord offset), so the attitude setpoint that
     spinal tracks stays level. */
  target_roll_ = 0;
  target_pitch_ = 0;

  Eigen::MatrixXd alloc = gimbalrotor_robot_model_->calcMaskedWrenchMatrixOnCoG(gimbal_dof_);
  Eigen::MatrixXd P_dash = Eigen::MatrixXd::Zero(4, input_num_);
  P_dash.row(0) = alloc.row(2);
  P_dash.bottomRows(3) = alloc.bottomRows(3);
  q_mat_inv_ = aerial_robot_model::pseudoinverse(P_dash);

  /* The LQI feedback is expressed in the CoG frame while the z error is in the world frame. The
     commanded part of that tilt is known exactly from the desired CoG orientation, so compensate it
     rather than relying on the (noisier) estimated attitude. */
  double des_r, des_p, des_y;
  robot_model_->getCogDesireOrientation<KDL::Rotation>().GetRPY(des_r, des_p, des_y);
  double tilt_scale = 1.0 / std::max(cos(des_r) * cos(des_p), 0.1);

  Eigen::VectorXd target_thrust_z_term = Eigen::VectorXd::Zero(input_num_);
  for(int i = 0; i < input_num_; i++)
    {
      double p_term = z_gains_.at(i)[0] * pid_controllers_.at(Z).getErrP();
      double i_term = z_gains_.at(i)[1] * pid_controllers_.at(Z).getErrI();
      double d_term = z_gains_.at(i)[2] * pid_controllers_.at(Z).getErrD();
      target_thrust_z_term(i) = (p_term + i_term + d_term) * tilt_scale;
      pid_msg_.z.p_term.at(i) = p_term;
      pid_msg_.z.i_term.at(i) = i_term;
      pid_msg_.z.d_term.at(i) = d_term;
    }

  /* feed-forward term for z */
  target_thrust_z_term += q_mat_inv_.col(0) * navigator_->getTargetAcc().z();

  /* constrain z (and its I term). The physical limit is on each rotor's thrust magnitude, which is
     the norm over that rotor's block of virtual inputs, not on a single component. */
  double max_term = 0;
  for(int i = 0; i < motor_num_; i++)
    max_term = std::max(max_term, target_thrust_z_term.segment(rotor_coef_ * i, rotor_coef_).norm());
  double residual = max_term - pid_controllers_.at(Z).getLimitSum();
  if(residual > 0 && max_term > 0)
    {
      pid_controllers_.at(Z).setErrI(pid_controllers_.at(Z).getPrevErrI());
      target_thrust_z_term *= (1 - residual / max_term);
    }

  for(int i = 0; i < input_num_; i++) pid_msg_.z.total.at(i) = target_thrust_z_term(i);

  /* x/y outer loop: allocate in the null space of z/roll/pitch/yaw, so the 40 Hz PC loop cannot
     perturb what spinal is stabilising at 1 kHz. */
  Eigen::VectorXd target_vectoring_f_xy = Eigen::VectorXd::Zero(input_num_);
  double alloc_det = (alloc * alloc.transpose()).determinant();
  if(alloc_det < allocation_det_thresh_)
    {
      ROS_WARN_THROTTLE_NAMED(1.0, "LQI gimbal control", "degenerate allocation (det: %f), suspend xy control", alloc_det);
      pid_controllers_.at(X).reset();
      pid_controllers_.at(Y).reset();
    }
  else
    {
      /* vertical acceleration currently produced by the z term, used to cancel the horizontal
         component the tilted thrust leaks into the world frame */
      double acc_z = (alloc.row(2) * target_thrust_z_term)(0);

      tf::Vector3 target_acc_w(pid_controllers_.at(X).result(), pid_controllers_.at(Y).result(), 0);
      tf::Vector3 target_acc_dash = (tf::Matrix3x3(tf::createQuaternionFromYaw(rpy_.z()))).inverse() * target_acc_w;

      Eigen::VectorXd target_wrench_acc = Eigen::VectorXd::Zero(6);
      target_wrench_acc(0) = target_acc_dash.x() - rpy_.y() * acc_z;
      target_wrench_acc(1) = target_acc_dash.y() + rpy_.x() * acc_z;
      target_vectoring_f_xy = aerial_robot_model::pseudoinverse(alloc) * target_wrench_acc;
    }

  /* Bound what we ask the gimbals for. Spinal turns each rotor's (lateral, thrust) pair into an
     angle, and the joint itself only travels +-2 rad, so an unbounded lateral command drives the
     servo into its stop and the allocation stops meaning anything. Scale the whole x/y contribution
     down uniformly (preserving its direction) until every rotor is within max_gimbal_angle. Note
     spinal still adds its own attitude and yaw terms on top, so this bounds the request, not the
     final angle. */
  double lateral_scale = 1.0;
  for(int i = 0; i < motor_num_; i++)
    {
      int lat = rotor_coef_ * i;
      int thr = rotor_coef_ * i + rotor_coef_ - 1;
      double lateral = target_vectoring_f_xy.segment(lat, rotor_coef_ - 1).norm();
      if(lateral < 1e-6) continue;

      double thrust = std::max(target_thrust_z_term(thr), 1e-3);
      double lateral_max = thrust * tan(max_gimbal_angle_);
      if(lateral > lateral_max) lateral_scale = std::min(lateral_scale, lateral_max / lateral);
    }
  if(lateral_scale < 1.0)
    {
      ROS_WARN_THROTTLE_NAMED(1.0, "LQI gimbal control", "xy command scaled by %f to keep the gimbals within %f rad",
                              lateral_scale, max_gimbal_angle_);
      target_vectoring_f_xy *= lateral_scale;
      /* the x/y demand could not be met, so stop the position integrators winding up against it */
      pid_controllers_.at(X).setErrI(pid_controllers_.at(X).getPrevErrI());
      pid_controllers_.at(Y).setErrI(pid_controllers_.at(Y).getPrevErrI());
    }

  Eigen::VectorXd target_f = target_thrust_z_term + target_vectoring_f_xy;

  /* Spinal resolves each rotor's gimbal angle as atan2(-f_lateral, f_thrust). Before the z integral
     has built up (on the ground) or while force-landing, f_thrust is near zero, so that angle is
     decided by noise and the gimbals whip around. Substitute the static hover thrust to keep the
     direction well defined, as the Dragon gimbal controller does. */
  if(!start_rp_integration_ || navigator_->getForceLandingFlag())
    {
      Eigen::VectorXd static_thrust = robot_model_->getStaticThrust();
      for(int i = 0; i < motor_num_; i++)
        target_f(rotor_coef_ * i + rotor_coef_ - 1) = static_thrust(i);
    }

  for(int i = 0; i < input_num_; i++) target_base_thrust_.at(i) = target_f(i);

  target_vectoring_f_ = target_f;

  /* the yaw term is reconstructed per input inside spinal, so only its scale is sent here */
  allocateYawTerm();
}

void GimbalLQIController::allocateYawTerm()
{
  Eigen::VectorXd target_thrust_yaw_term = Eigen::VectorXd::Zero(input_num_);
  for(int i = 0; i < input_num_; i++)
    {
      double p_term = yaw_gains_.at(i)[0] * pid_controllers_.at(YAW).getErrP();
      double i_term = yaw_gains_.at(i)[1] * pid_controllers_.at(YAW).getErrI();
      double d_term = yaw_gains_.at(i)[2] * pid_controllers_.at(YAW).getErrD();
      target_thrust_yaw_term(i) = p_term + i_term + d_term;
      pid_msg_.yaw.p_term.at(i) = p_term;
      pid_msg_.yaw.i_term.at(i) = i_term;
      pid_msg_.yaw.d_term.at(i) = d_term;
    }

  /* feed-forward term for yaw */
  target_thrust_yaw_term += q_mat_inv_.col(3) * navigator_->getTargetAngAcc().z();

  /* constrain yaw (and its I term) */
  double max_term = target_thrust_yaw_term.cwiseAbs().maxCoeff();
  double residual = max_term - pid_controllers_.at(YAW).getLimitSum();
  if(residual > 0 && max_term > 0)
    {
      pid_controllers_.at(YAW).setErrI(pid_controllers_.at(YAW).getPrevErrI());
      target_thrust_yaw_term *= (1 - residual / max_term);
    }

  /* only one scalar is sent because of the PC-spinal bandwidth; spinal rescales it per input by the
     ratio of the yaw d gains, so pick the entry whose gain is the reference. */
  double max_yaw_scale = 0;
  for(int i = 0; i < input_num_; i++)
    {
      pid_msg_.yaw.total.at(i) = target_thrust_yaw_term(i);

      if(yaw_gains_.at(i)[2] > max_yaw_scale)
        {
          max_yaw_scale = yaw_gains_.at(i)[2];
          candidate_yaw_term_ = target_thrust_yaw_term(i);
        }
    }
}

bool GimbalLQIController::update()
{
  /* spinal derives motor_number_ = uav_info.motor_num * (gimbal_dof + 1) and rejects an rpy/gain
     message whose length does not match, so this has to keep flowing regardless of flight state. */
  std_msgs::UInt8 gimbal_dof_msg;
  gimbal_dof_msg.data = gimbal_dof_;
  gimbal_dof_pub_.publish(gimbal_dof_msg);

  return UnderActuatedLQIController::update();
}

void GimbalLQIController::sendCmd()
{
  /* publishes FourAxisCommand: angles[0..1] = level attitude target, angles[2] = candidate yaw term,
     base_thrust = the input_num_ virtual inputs. Note that torque_allocation_matrix_inv is
     deliberately never published: spinal would overwrite the LQI gains with it. */
  UnderActuatedLQIController::sendCmd();

  std_msgs::Float32MultiArray target_vectoring_force_msg;
  for(int i = 0; i < target_vectoring_f_.size(); i++)
    target_vectoring_force_msg.data.push_back(target_vectoring_f_(i));
  target_vectoring_force_pub_.publish(target_vectoring_force_msg);
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::GimbalLQIController, aerial_robot_control::ControlBase);
