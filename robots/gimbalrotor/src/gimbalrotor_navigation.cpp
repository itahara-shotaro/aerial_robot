// -*- mode: c++ -*-

#include <gimbalrotor/gimbalrotor_navigation.h>

using namespace aerial_robot_model;
using namespace aerial_robot_navigation;

GimbalrotorNavigator::GimbalrotorNavigator()
  : BaseNavigator(),
    eq_cog_world_(false),
    roll_nav_mode_(aerial_robot_msgs::FlightNav::NO_NAVIGATION),
    pitch_nav_mode_(aerial_robot_msgs::FlightNav::NO_NAVIGATION),
    roll_nav_reset_time_(0),
    pitch_nav_reset_time_(0)
{
  curr_target_baselink_rot_.setRPY(0, 0, 0);
  final_target_baselink_rot_.setRPY(0, 0, 0);
}

void GimbalrotorNavigator::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                      boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                      boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                      double loop_du)
{
  /* initialize the flight control */
  BaseNavigator::initialize(nh, nhp, robot_model, estimator, loop_du);

  target_baselink_rpy_pub_ = nh_.advertise<spinal::DesireCoord>("desire_coordinate", 1);  // to spinal
  final_target_baselink_rot_sub_ =
      nh_.subscribe("final_target_baselink_rot", 1, &GimbalrotorNavigator::targetBaselinkRotCallback, this);
  final_target_baselink_rpy_sub_ =
      nh_.subscribe("final_target_baselink_rpy", 1, &GimbalrotorNavigator::targetBaselinkRPYCallback, this);
  prev_rotation_stamp_ = ros::Time::now().toSec();
}

void GimbalrotorNavigator::update()
{
  BaseNavigator::update();

  const double now = ros::Time::now().toSec();
  if (roll_nav_mode_ == aerial_robot_msgs::FlightNav::VEL_MODE)
  {
    if (now <= roll_nav_reset_time_)
      setTargetRoll(getTargetRPY().x() + getTargetOmega().x() * loop_du_);
    else
    {
      setTargetOmegaX(0);
      roll_nav_mode_ = aerial_robot_msgs::FlightNav::POS_MODE;
    }
  }
  else if (roll_nav_mode_ == aerial_robot_msgs::FlightNav::POS_VEL_MODE && now > roll_nav_reset_time_)
  {
    setTargetOmegaX(0);
    roll_nav_mode_ = aerial_robot_msgs::FlightNav::POS_MODE;
  }

  if (pitch_nav_mode_ == aerial_robot_msgs::FlightNav::VEL_MODE)
  {
    if (now <= pitch_nav_reset_time_)
      setTargetPitch(getTargetRPY().y() + getTargetOmega().y() * loop_du_);
    else
    {
      setTargetOmegaY(0);
      pitch_nav_mode_ = aerial_robot_msgs::FlightNav::POS_MODE;
    }
  }
  else if (pitch_nav_mode_ == aerial_robot_msgs::FlightNav::POS_VEL_MODE && now > pitch_nav_reset_time_)
  {
    setTargetOmegaY(0);
    pitch_nav_mode_ = aerial_robot_msgs::FlightNav::POS_MODE;
  }

  baselinkRotationProcess();
}

void GimbalrotorNavigator::reset()
{
  BaseNavigator::reset();

  // reset SO3
  eq_cog_world_ = false;
  curr_target_baselink_rot_.setRPY(0, 0, 0);
  final_target_baselink_rot_.setRPY(0, 0, 0);
  roll_nav_mode_ = aerial_robot_msgs::FlightNav::NO_NAVIGATION;
  pitch_nav_mode_ = aerial_robot_msgs::FlightNav::NO_NAVIGATION;
  roll_nav_reset_time_ = 0;
  pitch_nav_reset_time_ = 0;
  KDL::Rotation rot;
  tf::quaternionTFToKDL(curr_target_baselink_rot_, rot);
  robot_model_->setCogDesireOrientation(rot);
}

void GimbalrotorNavigator::targetBaselinkRotCallback(const geometry_msgs::QuaternionStampedConstPtr& msg)
{
  tf::quaternionMsgToTF(msg->quaternion, final_target_baselink_rot_);
  target_omega_.setValue(0, 0, 0);  // for sure to reset the target angular velocity

  // special process
  if (getTargetRPY().z() != 0)
  {
    curr_target_baselink_rot_.setRPY(0, 0, getTargetRPY().z());
    eq_cog_world_ = true;
  }
}

void GimbalrotorNavigator::targetBaselinkRPYCallback(const geometry_msgs::Vector3StampedConstPtr& msg)
{
  final_target_baselink_rot_.setRPY(msg->vector.x, msg->vector.y, msg->vector.z);
  target_omega_.setValue(0, 0, 0);  // for sure to reset the target angular velocity
}

void GimbalrotorNavigator::naviCallback(const aerial_robot_msgs::FlightNavConstPtr& msg)
{
  if (getNaviState() != HOVER_STATE)
    return;

  BaseNavigator::naviCallback(msg);

  const double now = ros::Time::now().toSec();
  switch (msg->roll_nav_mode)
  {
    case aerial_robot_msgs::FlightNav::POS_MODE:
      setTargetRoll(msg->target_roll);
      setTargetOmegaX(0);
      roll_nav_mode_ = msg->roll_nav_mode;
      break;
    case aerial_robot_msgs::FlightNav::VEL_MODE:
      setTargetOmegaX(msg->target_omega_x);
      roll_nav_mode_ = msg->roll_nav_mode;
      roll_nav_reset_time_ = now + teleop_reset_duration_;
      break;
    case aerial_robot_msgs::FlightNav::POS_VEL_MODE:
      setTargetRoll(msg->target_roll);
      setTargetOmegaX(msg->target_omega_x);
      roll_nav_mode_ = msg->roll_nav_mode;
      roll_nav_reset_time_ = now + trajectory_reset_duration_;
      break;
    default:
      break;
  }

  switch (msg->pitch_nav_mode)
  {
    case aerial_robot_msgs::FlightNav::POS_MODE:
      setTargetPitch(msg->target_pitch);
      setTargetOmegaY(0);
      pitch_nav_mode_ = msg->pitch_nav_mode;
      break;
    case aerial_robot_msgs::FlightNav::VEL_MODE:
      setTargetOmegaY(msg->target_omega_y);
      pitch_nav_mode_ = msg->pitch_nav_mode;
      pitch_nav_reset_time_ = now + teleop_reset_duration_;
      break;
    case aerial_robot_msgs::FlightNav::POS_VEL_MODE:
      setTargetPitch(msg->target_pitch);
      setTargetOmegaY(msg->target_omega_y);
      pitch_nav_mode_ = msg->pitch_nav_mode;
      pitch_nav_reset_time_ = now + trajectory_reset_duration_;
      break;
    default:
      break;
  }
}

void GimbalrotorNavigator::baselinkRotationProcess()
{
  if (curr_target_baselink_rot_ == final_target_baselink_rot_)
    return;

  if (ros::Time::now().toSec() - prev_rotation_stamp_ > baselink_rot_pub_interval_)
  {
    tf::Quaternion delta_q = curr_target_baselink_rot_.inverse() * final_target_baselink_rot_;
    double angle = delta_q.getAngle();
    if (angle > M_PI)
      angle -= 2 * M_PI;

    if (fabs(angle) > baselink_rot_change_thresh_)
    {
      curr_target_baselink_rot_ *= tf::Quaternion(delta_q.getAxis(), fabs(angle) / angle * baselink_rot_change_thresh_);
    }
    else
      curr_target_baselink_rot_ = final_target_baselink_rot_;

    KDL::Rotation rot;
    tf::quaternionTFToKDL(curr_target_baselink_rot_, rot);
    robot_model_->setCogDesireOrientation(rot);

    // send to spinal
    spinal::DesireCoord msg;
    double r, p, y;
    tf::Matrix3x3(curr_target_baselink_rot_).getRPY(r, p, y);
    msg.roll = r;
    msg.pitch = p;
    msg.yaw = y;
    target_baselink_rpy_pub_.publish(msg);

    prev_rotation_stamp_ = ros::Time::now().toSec();
  }
}

void GimbalrotorNavigator::rosParamInit()
{
  BaseNavigator::rosParamInit();

  ros::NodeHandle navi_nh(nh_, "navigation");

  getParam<double>(navi_nh, "baselink_rot_change_thresh", baselink_rot_change_thresh_,
                   0.02);  // the threshold to change the baselink rotation
  getParam<double>(navi_nh, "baselink_rot_pub_interval", baselink_rot_pub_interval_,
                   0.1);  // the rate to pub baselink rotation command
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_navigation::GimbalrotorNavigator, aerial_robot_navigation::BaseNavigator);
