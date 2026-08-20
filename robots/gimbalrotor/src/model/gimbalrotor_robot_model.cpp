#include <gimbalrotor/model/gimbalrotor_robot_model.h>
#include <aerial_robot_model/utils/math_utils.h>
#include <tf_conversions/tf_kdl.h>
#include <tf_conversions/tf_eigen.h>

GimbalrotorRobotModel::GimbalrotorRobotModel(bool init_with_rosparam, bool verbose, double fc_t_min_thre, double epsilon) :
  RobotModel(init_with_rosparam, verbose, fc_t_min_thre, epsilon)
{
  const int rotor_num = getRotorNum();
  const int joint_num = getJointNum();

  links_rotation_from_cog_.resize(rotor_num);
  thrust_coords_rot_.resize(rotor_num);
}

void GimbalrotorRobotModel::updateRobotModelImpl(const KDL::JntArray& joint_positions)
{
  KDL::TreeFkSolverPos_recursive fk_solver(getTree());
  // /* special process */
  KDL::Frame f_baselink;
  fk_solver.JntToCart(joint_positions, f_baselink, getBaselinkName());
  const KDL::Rotation cog_frame = f_baselink.M * getCogDesireOrientation<KDL::Rotation>().Inverse();
  transformable::RobotModel::updateRobotModelImpl(joint_positions);
  const auto seg_tf_map = getSegmentsTf();

  /* get local coords of thrust links */
  for(int i = 0; i < getRotorNum(); ++i)
    {
      std::string thrust = "rotor_arm" + std::to_string(i + 1);
      KDL::Frame f;
      fk_solver.JntToCart(joint_positions, f, thrust);
      thrust_coords_rot_[i] = cog_frame.Inverse() * f.M;

    }
}

Eigen::MatrixXd GimbalrotorRobotModel::calcMaskedWrenchMatrixOnCoG(int gimbal_dof)
{
  const int rotor_num = getRotorNum();
  const int rotor_coef = gimbal_dof + 1;

  const double mass_inv = 1 / getMass();
  const Eigen::Matrix3d inertia_inv = getInertia<Eigen::Matrix3d>().inverse();
  const std::vector<Eigen::Vector3d> rotors_origin_from_cog = getRotorsOriginFromCog<Eigen::Vector3d>();
  const auto& rotor_direction = getRotorDirection();
  const double m_f_rate = getMFRate();

  /* full allocation: one free 3-D force vector per rotor */
  Eigen::MatrixXd full_q_mat = Eigen::MatrixXd::Zero(6, 3 * rotor_num);
  Eigen::MatrixXd wrench_map = Eigen::MatrixXd::Zero(6, 3);
  wrench_map.block(0, 0, 3, 3) = Eigen::MatrixXd::Identity(3, 3);
  int last_col = 0;
  for(int i = 0; i < rotor_num; i++){
    wrench_map.block(3, 0, 3, 3) = aerial_robot_model::skew(rotors_origin_from_cog.at(i)) + rotor_direction.at(i + 1) * m_f_rate * Eigen::Matrix3d::Identity();
    full_q_mat.middleCols(last_col, 3) = wrench_map;
    last_col += 3;
  }
  full_q_mat.topRows(3) = mass_inv * full_q_mat.topRows(3);
  full_q_mat.bottomRows(3) = inertia_inv * full_q_mat.bottomRows(3);

  /* mask each rotor's force to the subspace its gimbal can reach */
  std::vector<KDL::Rotation> thrust_coords_rot = getThrustCoordRot<KDL::Rotation>();
  Eigen::MatrixXd integrated_rot = Eigen::MatrixXd::Zero(3 * rotor_num, rotor_coef * rotor_num);
  for(int i = 0; i < rotor_num; i++){
    tf::Quaternion r;  tf::quaternionKDLToTF(thrust_coords_rot.at(i), r);
    Eigen::Matrix3d conv_cog_from_thrust; tf::matrixTFToEigen(tf::Matrix3x3(r), conv_cog_from_thrust);
    Eigen::MatrixXd mask;
    if(gimbal_dof == 1)
      {
        mask = Eigen::MatrixXd::Zero(3, 2);
        mask << 0, 0, 1, 0, 0, 1;
      }
    else
      {
        mask = Eigen::MatrixXd::Identity(3, 3);
      }
    integrated_rot.block(3 * i, rotor_coef * i, 3, rotor_coef) = conv_cog_from_thrust * mask;
  }

  return full_q_mat * integrated_rot;
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(GimbalrotorRobotModel, aerial_robot_model::RobotModel);
