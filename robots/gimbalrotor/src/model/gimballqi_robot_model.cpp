#include <gimbalrotor/model/gimballqi_robot_model.h>

GimbalLQIRobotModel::GimbalLQIRobotModel(bool init_with_rosparam, bool verbose, double fc_t_min_thre, double epsilon) :
  RobotModel(init_with_rosparam, verbose, fc_t_min_thre, epsilon)
{
  const int rotor_num = getRotorNum();
  const int joint_num = getJointNum();

  links_rotation_from_cog_.resize(rotor_num);
  thrust_coords_rot_.resize(rotor_num);
}

void GimbalLQIRobotModel::updateRobotModelImpl(const KDL::JntArray& joint_positions)
{
  KDL::TreeFkSolverPos_recursive fk_solver(getTree());
  // /* special process */
  KDL::Frame f_baselink;
  fk_solver.JntToCart(joint_positions, f_baselink, getBaselinkName());
  const KDL::Rotation cog_frame = f_baselink.M * getCogDesireOrientation<KDL::Rotation>().Inverse();
  transformable::RobotModel::updateRobotModelImpl(joint_positions);
  const auto seg_tf_map = getSegmentsTf();

  const auto joint_index_map = getJointIndexMap();
  gimbal_processed_joint_ = joint_positions;

  /* get local coords of thrust links */
  for(int i = 0; i < getRotorNum(); ++i)
    {
      std::string s = std::to_string(i + 1);
      std::string thrust = "rotor_arm" + s;
      KDL::Frame f;
      fk_solver.JntToCart(joint_positions, f, thrust);
      thrust_coords_rot_[i] = cog_frame.Inverse() * f.M;
      double r, p, y;
      thrust_coords_rot_[i].GetRPY(r, p, y);
      gimbal_processed_joint_(joint_index_map.find(std::string("gimbal") + s )->second) = -r;
      // gimbal_processed_joint_(joint_index_map.find(std::string("gimbal") + s + std::string("_pitch"))->second) = -p;

      gimbal_nominal_angles_[i] = -r;
      // gimbal_nominal_angles_[i * 2 + 1] = -p;


    }
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(GimbalLQIRobotModel, aerial_robot_model::RobotModel);
