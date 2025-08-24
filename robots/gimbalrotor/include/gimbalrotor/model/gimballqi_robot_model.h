// -*- mode: c++ -*-

#pragma once

#include <aerial_robot_model/model/transformable_aerial_robot_model.h>

using namespace aerial_robot_model;

class GimbalLQIRobotModel : public transformable::RobotModel{
public:
  GimbalLQIRobotModel(bool init_with_rosparam = true,
                    bool verbose = false,
                    double fc_t_min_thre = 0,
                    double epsilon = 10);
  virtual ~GimbalLQIRobotModel() = default;

  template <class T> std::vector<T> getLinksRotationFromCog();
  template <class T> std::vector<T> getThrustCoordRot();

private:
  void updateRobotModelImpl(const KDL::JntArray& joint_positions) override;

  KDL::JntArray gimbal_processed_joint_;
  std::vector<KDL::Rotation> links_rotation_from_cog_;
  std::vector<KDL::Rotation> thrust_coords_rot_;
  std::vector<double> gimbal_nominal_angles_;
  std::mutex gimbal_nominal_angles_mutex_;


  std::mutex links_rotation_mutex_;
  std::mutex thrust_rotation_mutex_;

  const std::vector<double> getGimbalNominalAngles()
  {
    std::lock_guard<std::mutex> lock(gimbal_nominal_angles_mutex_);
    return gimbal_nominal_angles_;
  }

  void setGimbalNominalAngles(const std::vector<double> gimbal_nominal_angles)
  {
    std::lock_guard<std::mutex> lock(gimbal_nominal_angles_mutex_);
    gimbal_nominal_angles_ = gimbal_nominal_angles;
  }

};

template<> inline std::vector<KDL::Rotation> GimbalLQIRobotModel::getLinksRotationFromCog()
{
  std::lock_guard<std::mutex> lock(links_rotation_mutex_);
  return links_rotation_from_cog_;
}

template<> inline std::vector<KDL::Rotation> GimbalLQIRobotModel::getThrustCoordRot()
{
  std::lock_guard<std::mutex> lock(thrust_rotation_mutex_);
  return thrust_coords_rot_;
}
