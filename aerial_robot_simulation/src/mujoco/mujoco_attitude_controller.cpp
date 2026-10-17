#include <aerial_robot_simulation/mujoco/mujoco_attitude_controller.h>

namespace flight_controllers
{
  MujocoAttitudeController::MujocoAttitudeController():
    controller_core_(new FlightControl())
  {
  }

  bool MujocoAttitudeController::init(hardware_interface::MujocoSpinalInterface *robot, ros::NodeHandle &n)
  {
    spinal_interface_ = robot;
    motor_num_ = spinal_interface_->getMotorNum();

    int index = n.getNamespace().rfind('/');
    std::string robot_ns = n.getNamespace().substr(0, index);
    ros::NodeHandle n_robot = ros::NodeHandle(robot_ns);
    controller_core_->init(&n_robot, robot->getEstimatorPtr());

    std::string full_param;
    int estimate_mode;
    if (n.searchParam("estimation/mode", full_param) && n.getParam(full_param, estimate_mode))
      spinal_interface_->useGroundTruth(estimate_mode == aerial_robot_estimation::GROUND_TRUTH);
    else
    {
      ROS_WARN_STREAM_NAMED("mujoco_attitude_controller", "Cannot find estimation/mode in "
                            << n.getNamespace() << "; using ground truth");
      spinal_interface_->useGroundTruth(true);
    }

    return true;

  }

  void MujocoAttitudeController::starting(const ros::Time& time)
  {
  }

  void MujocoAttitudeController::update(const ros::Time& time, const ros::Duration& period)
  {
    /* freeze the attitude estimator while touching the ground, since the bad contact simulation performance in gazebo */
    spinal_interface_->onGround(!controller_core_->getAttController().getIntegrateFlag());

    /* update the controller */
    controller_core_->update();

    for(int i = 0; i < motor_num_; i++)
      {
        spinal_interface_->setForce(i, controller_core_->getAttController().getForce(i));
      }
  }
}


PLUGINLIB_EXPORT_CLASS(flight_controllers::MujocoAttitudeController, controller_interface::ControllerBase)
