#include <hydrus_xi/hydrus_xi_fully_actuated_robot_model.h>


HydrusXiFullyActuatedRobotModel::HydrusXiFullyActuatedRobotModel(bool init_with_rosparam, bool verbose, double epsilon) :
  RobotModel(init_with_rosparam, verbose), epsilon_(epsilon)
{

  if (init_with_rosparam) {
    getParamFromRos();
  }

  const int rotor_num = getRotorNum();
  const int joint_num = getJointNum();
  const int full_body_ndof = 6 + joint_num;
  u_jacobian_.resize(rotor_num);
  v_jacobian_.resize(rotor_num);
  p_jacobian_.resize(rotor_num);
  q_jacobian_.resize(full_body_ndof);
  cog_jacobian_.resize(3, full_body_ndof);
  gravity_.resize(6);
  gravity_ <<  0, 0, 9.80665, 0, 0, 0;
  gravity_3d_.resize(3);
  gravity_3d_ << 0, 0, 9.80665;
  lambda_jacobian_.resize(rotor_num, full_body_ndof);
  f_min_ij_.resize(rotor_num * (rotor_num - 1));
  t_min_ij_.resize(rotor_num * (rotor_num - 1));
  f_min_jacobian_.resize(rotor_num * (rotor_num - 1), full_body_ndof);
  t_min_jacobian_.resize(rotor_num * (rotor_num - 1), full_body_ndof);
  thrust_coord_jacobians_.resize(rotor_num);
  cog_coord_jacobians_.resize(getInertiaMap().size());
  joint_torque_.resize(joint_num);
  joint_torque_jacobian_.resize(joint_num, full_body_ndof);
  static_thrust_.resize(rotor_num);
  thrust_wrench_.resize(rotor_num);

  fr_root_jacobians_.resize(rotor_num); // debug
  fr_wrench_units_.resize(rotor_num); // debug

  u_triple_product_jacobian_.resize(rotor_num);
  for (auto& j : u_triple_product_jacobian_) {
    j.resize(rotor_num);
    for (auto& k : j) {
      k.resize(rotor_num);
      for (auto& vec : k) {
        vec.resize(full_body_ndof);
      }
    }
  }

  v_triple_product_jacobian_.resize(rotor_num);
  for (auto& j : v_triple_product_jacobian_) {
    j.resize(rotor_num);
    for (auto& k : j) {
      k.resize(rotor_num);
      for (auto& vec : k) {
        vec.resize(full_body_ndof);
      }
    }
  }
}


void HydrusXiFullyActuatedRobotModel::getParamFromRos()
{
  ros::NodeHandle nhp("~");
  nhp.param("epslion", epsilon_, 10.0);
}

void HydrusXiFullyActuatedRobotModel::updateJacobians(const KDL::JntArray& joint_positions)
{
  //  updateRobotModel(joint_positions); //TODO

  const int joint_num = getJointNum();
  const int full_body_ndof = 6 + joint_num;
  const int rotor_num = getRotorNum();
  const auto& inertia_map = getInertiaMap();
  const double thrust_max = getThrustUpperLimit();
  const double m_f_rate = getMFRate();
  const auto& u = getRotorsNormalFromCog<Eigen::Vector3d>();
  const auto& p = getRotorsOriginFromCog<Eigen::Vector3d>();
  const auto& sigma = getRotorDirection();
  std::vector<Eigen::Vector3d> v(rotor_num);
  Eigen::MatrixXd q_mat = calcQMatrix(u, p, sigma);
  Eigen::MatrixXd q_inv = aerial_robot_model::pseudoinverse(q_mat);
  Eigen::Vector3d fg = getMass() * gravity_3d_;

  calcCOGJacobian();

  //calc jacobian of u(thrust direction, force vector), p(thrust position), v(torque vector)
  for (int i = 0; i < rotor_num; ++i) {
    std::string seg_name = std::string("thrust") + std::to_string(i + 1);
    Eigen::MatrixXd thrust_coord_jacobian = getJacobian(joint_positions, seg_name);
    thrust_coord_jacobians_.at(i) = thrust_coord_jacobian;
    u_jacobian_.at(i) = -skew(u.at(i)) * thrust_coord_jacobian.bottomRows(3);
    p_jacobian_.at(i) = thrust_coord_jacobian.topRows(3) - cog_jacobian_;
    v.at(i) = p.at(i).cross(u.at(i)) + m_f_rate * sigma.at(i + 1) * u.at(i);
    v_jacobian_.at(i) = -skew(u.at(i)) * p_jacobian_.at(i) + skew(p.at(i)) * u_jacobian_.at(i) + m_f_rate * sigma.at(i + 1) * u_jacobian_.at(i);
  }

  //calc jacobian of q(matrix), lambda(thrust value)
  Eigen::MatrixXd temp_jacobian = Eigen::MatrixXd::Zero(6, full_body_ndof);
  for (int i = 0; i < full_body_ndof; ++i) {
    Eigen::MatrixXd q_jacobi_i(6, rotor_num);
    for (int j = 0; j < rotor_num; ++j) {
      q_jacobi_i.block(0, j, 3, 1) = u_jacobian_.at(j).col(i);
      q_jacobi_i.block(3, j, 3, 1) = v_jacobian_.at(j).col(i);
    }
    q_jacobian_.at(i) = q_jacobi_i;
    lambda_jacobian_.col(i) = -q_inv * q_jacobi_i * q_inv * getMass() * gravity_;
    temp_jacobian.col(i) = q_jacobi_i * q_inv * getMass() * gravity_;
  }
  //ROS_INFO_STREAM("temp_jacobian w.r.t cog: \n" << temp_jacobian);

  lambda_jacobian_ = calcLambdaJacobain(joint_positions); // replace

  //calc jacobian of joint torque
  static_thrust_ = calcStaticThrust(q_inv);
  joint_torque_ = Eigen::VectorXd::Zero(joint_num);
  joint_torque_jacobian_ = Eigen::MatrixXd::Zero(joint_num, full_body_ndof);

  // update coord jacobians for cog point and convert to joint torque
  int seg_index = 0;
  for(const auto& inertia : inertia_map)
    {
      cog_coord_jacobians_.at(seg_index) = getJacobian(joint_positions, inertia.first, inertia.second.getCOG());
      joint_torque_ -= cog_coord_jacobians_.at(seg_index).rightCols(joint_num).transpose() * inertia.second.getMass() * (-gravity_);

      for (int j = 0; j < joint_num; ++j) {
        joint_torque_jacobian_.row(j) += inertia.second.getMass() * (-gravity_.transpose()) * getSecondDerivative(inertia.first, j, inertia.second.getCOG());
      }
      seg_index ++;
    }

  // TODO: update coord jacobians for external force

  // thrust
  for (int i = 0; i < rotor_num; ++i) {
    Eigen::VectorXd wrench(6);
    Eigen::Vector3d thrust = u.at(i) * static_thrust_(i);
    wrench.block(0, 0, 3, 1) = thrust;
    wrench.block(3, 0, 3, 1) = m_f_rate * sigma.at(i + 1) * thrust;
    std::string thrust_name = std::string("thrust") + std::to_string(i + 1);

    joint_torque_ -= thrust_coord_jacobians_.at(i).rightCols(joint_num).transpose() * wrench;
    thrust_wrench_.at(i) = wrench;

    // fix bug: not using hessian
    for (int j = 0; j < joint_num; ++j) {
      joint_torque_jacobian_.row(j) += wrench.transpose() * getSecondDerivative(thrust_name, j);
      }

    Eigen::MatrixXd wrench_jacobian(6, full_body_ndof);
    wrench_jacobian.topRows(3) = u_jacobian_.at(i) * static_thrust_(i);
    wrench_jacobian.bottomRows(3) = m_f_rate * sigma.at(i+1) * u_jacobian_.at(i) * static_thrust_(i); // fix bug

    joint_torque_jacobian_ += thrust_coord_jacobians_.at(i).rightCols(joint_num).transpose() * (wrench_jacobian + wrench / static_thrust_(i) * lambda_jacobian_.row(i));
  }

  joint_torque_jacobian_ *= -1;

  return; // temp

  //calc jacobian of f_min_ij, t_min_ij
  int f_min_index = 0;
  for (int i = 0; i < rotor_num; ++i) {
    for (int j = 0; j < rotor_num; ++j) {
      double f_min = 0.0;
      double t_min = 0.0;
      Eigen::VectorXd d_f_min = Eigen::VectorXd::Zero(full_body_ndof);
      Eigen::VectorXd d_t_min = Eigen::VectorXd::Zero(full_body_ndof);
      const Eigen::Vector3d& u_i = u.at(i);
      const Eigen::Vector3d& u_j = u.at(j);
      const Eigen::Vector3d uixuj = u_i.cross(u_j);
      const Eigen::Vector3d& v_i = v.at(i);
      const Eigen::Vector3d& v_j = v.at(j);
      const Eigen::Vector3d vixvj = v_i.cross(v_j);

      for (int k = 0; k < rotor_num; ++k) {
        if (i == j || j == k || k == i) {
          u_triple_product_jacobian_.at(i).at(j).at(k) = Eigen::VectorXd::Zero(full_body_ndof);
          v_triple_product_jacobian_.at(i).at(j).at(k) = Eigen::VectorXd::Zero(full_body_ndof);
        } else {
          const Eigen::Vector3d& u_k = u.at(k);
          const Eigen::Vector3d& v_k = v.at(k);
          const double u_triple_product = calcTripleProduct(u_i, u_j, u_k);
          const double v_triple_product = calcTripleProduct(v_i, v_j, v_k);
          for (int l = 0; l < full_body_ndof; ++l) {
            {
              const Eigen::Vector3d& d_u_i = u_jacobian_.at(i).col(l);
              const Eigen::Vector3d& d_u_j = u_jacobian_.at(j).col(l);
              const Eigen::Vector3d& d_u_k = u_jacobian_.at(k).col(l);
              const Eigen::Vector3d d_uixuj = u_i.cross(d_u_j) + d_u_i.cross(u_j);

              double d_u_triple_product = (uixuj / uixuj.norm()).dot(d_u_k) + u_k.dot(1/uixuj.norm() * d_uixuj - uixuj / (uixuj.norm() * uixuj.squaredNorm()) * uixuj.dot(d_uixuj));
              u_triple_product_jacobian_.at(i).at(j).at(k)(l) = d_u_triple_product;
              d_f_min(l) += sigmoid(u_triple_product * thrust_max, epsilon_) * d_u_triple_product * thrust_max;
            }
            {
              const Eigen::Vector3d& d_v_i = v_jacobian_.at(i).col(l);
              const Eigen::Vector3d& d_v_j = v_jacobian_.at(j).col(l);
              const Eigen::Vector3d& d_v_k = v_jacobian_.at(k).col(l);
              const Eigen::Vector3d d_vixvj = v_i.cross(d_v_j) + d_v_i.cross(v_j);

              double d_v_triple_product = (vixvj / vixvj.norm()).dot(d_v_k) + v_k.dot(1/vixvj.norm() * d_vixvj - vixvj / (vixvj.norm() * vixvj.squaredNorm()) * vixvj.dot(d_vixvj));
              v_triple_product_jacobian_.at(i).at(j).at(k)(l) = d_v_triple_product;
              d_t_min(l) += sigmoid(v_triple_product * thrust_max, epsilon_) * d_v_triple_product * thrust_max;
            }
          } //l
          f_min += reluApprox(u_triple_product * thrust_max, epsilon_);
          t_min += reluApprox(v_triple_product * thrust_max, epsilon_);
        } //if
      } //k

      if (i != j) {
        double uixuj_fg = uixuj.dot(fg)/uixuj.norm();
        Eigen::VectorXd d_uixuj_fg(full_body_ndof);
        for (int l = 0; l < full_body_ndof; ++l) {
          const Eigen::Vector3d& d_u_i = u_jacobian_.at(i).col(l);
          const Eigen::Vector3d& d_u_j = u_jacobian_.at(j).col(l);
          const Eigen::Vector3d d_uixuj = u_i.cross(d_u_j) + d_u_i.cross(u_j);
          d_uixuj_fg(l) = fg.dot(1/uixuj.norm() * d_uixuj - uixuj / (uixuj.norm() * uixuj.squaredNorm()) * uixuj.dot(d_uixuj));
        } //l
        f_min_ij_(f_min_index) = absApprox(f_min - uixuj_fg, epsilon_);
        t_min_ij_(f_min_index) = absApprox(t_min, epsilon_);
        f_min_jacobian_.row(f_min_index) = (tanh(f_min - uixuj_fg, epsilon_) * (d_f_min - d_uixuj_fg)).transpose();
        t_min_jacobian_.row(f_min_index) = (tanh(t_min, epsilon_) * (d_t_min)).transpose();
        f_min_index++;
      }
    } //j
  } //i
}

void HydrusXiFullyActuatedRobotModel::updateJacobians(const sensor_msgs::JointState& joint_state)
{
  updateJacobians(jointMsgToKdl(joint_state));
}

Eigen::MatrixXd HydrusXiFullyActuatedRobotModel::getJacobian(const KDL::JntArray& joint_positions, std::string segment_name, KDL::Vector offset)
{
  const auto& tree = getTree();
  const auto& seg_frames = getSegmentsTf();
  const auto root_rot = getCogDesireOrientation<Eigen::Matrix3d>();

  KDL::TreeJntToJacSolver solver(tree);
  KDL::Jacobian jac(tree.getNrOfJoints());
  int status = solver.JntToJac(joint_positions, jac, segment_name);
  jac.changeRefPoint(seg_frames.at(segment_name).M * offset);

  // joint part
  Eigen::MatrixXd jac_joint = convertJacobian(jac.data);
  // return jac_joint; // only joint jacobian

  // add virtual 6dof root
  Eigen::MatrixXd jac_all = Eigen::MatrixXd::Identity(6, 6 + getJointNum());
  jac_all.rightCols(getJointNum()) = jac_joint;
  Eigen::Vector3d p = aerial_robot_model::kdlToEigen(seg_frames.at(segment_name).p + seg_frames.at(segment_name).M * offset);
  jac_all.block(0,3,3,3) = -aerial_robot_model::skew(p);

  jac_all.topRows(3) = root_rot * jac_all.topRows(3);
  jac_all.bottomRows(3) = root_rot * jac_all.bottomRows(3);
  return jac_all;

}

// @deprecated
Eigen::VectorXd HydrusXiFullyActuatedRobotModel::getHessian(std::string ref_frame, int joint_i, int joint_j, KDL::Vector offset)
{
  const auto& segment_map = getTree().getSegments();
  const auto& seg_frames = getSegmentsTf();
  const auto& joint_hierachy = getJointHierachy();
  const auto& joint_segment_map = getJointSegmentMap();
  const auto& joint_names = getJointNames();
  const auto& joint_parent_link_names = getJointParentLinkNames();
  const auto root_rot = getCogDesireOrientation<Eigen::Matrix3d>();

  Eigen::Vector3d p_e = aerial_robot_model::kdlToEigen(seg_frames.at(ref_frame).p + seg_frames.at(ref_frame).M * offset);

  std::string joint_i_name = joint_names.at(joint_i);
  std::string joint_j_name = joint_names.at(joint_j);

  if (joint_hierachy.at(joint_i_name) > joint_hierachy.at(joint_j_name)) {
    std::swap(joint_i_name, joint_j_name);
  }

  std::vector<std::string> joint_i_child_segments = joint_segment_map.at(joint_i_name);
  std::vector<std::string> joint_j_child_segments = joint_segment_map.at(joint_j_name);

  if (std::find(joint_i_child_segments.begin(), joint_i_child_segments.end(), ref_frame) == joint_i_child_segments.end() ||  std::find(joint_j_child_segments.begin(), joint_j_child_segments.end(), ref_frame) == joint_j_child_segments.end()) {
    return Eigen::VectorXd::Zero(6);
  }

  std::string joint_i_child_segment_name = joint_i_child_segments.at(0);
  std::string joint_j_child_segment_name = joint_j_child_segments.at(0);
  const KDL::Segment& joint_i_child_segment = GetTreeElementSegment(segment_map.at(joint_i_child_segment_name));
  const KDL::Segment& joint_j_child_segment = GetTreeElementSegment(segment_map.at(joint_j_child_segment_name));
  Eigen::Vector3d a_i = aerial_robot_model::kdlToEigen(seg_frames.at(joint_parent_link_names.at(joint_i)).M * joint_i_child_segment.getJoint().JointAxis());
  Eigen::Vector3d a_j = aerial_robot_model::kdlToEigen(seg_frames.at(joint_parent_link_names.at(joint_j)).M * joint_j_child_segment.getJoint().JointAxis());
  Eigen::Vector3d p_j = aerial_robot_model::kdlToEigen(seg_frames.at(joint_j_child_segment_name).p); //joint pos

  Eigen::VectorXd derivative(6);
  derivative.topRows(3) = root_rot * a_i.cross(a_j.cross(p_e - p_j));
  derivative.bottomRows(3) = root_rot * a_i.cross(a_j);

  return derivative;
}

Eigen::MatrixXd HydrusXiFullyActuatedRobotModel::getSecondDerivative(std::string ref_frame, int joint_i, KDL::Vector offset)
{
  const auto& segment_map = getTree().getSegments();
  const auto& seg_frames = getSegmentsTf();
  const auto& joint_hierachy = getJointHierachy();
  const auto& joint_segment_map = getJointSegmentMap();
  const auto& joint_names = getJointNames();
  const auto& joint_parent_link_names = getJointParentLinkNames();
  const auto& joint_num = getJointNum();
  const int full_body_ndof = 6 + joint_num;
  const auto root_rot = getCogDesireOrientation<Eigen::Matrix3d>();

  Eigen::Vector3d p_e = aerial_robot_model::kdlToEigen(seg_frames.at(ref_frame).p + seg_frames.at(ref_frame).M * offset);

  std::string joint_i_name = joint_names.at(joint_i);
  std::vector<std::string> joint_i_child_segments = joint_segment_map.at(joint_i_name);
  std::string joint_i_child_segment_name = joint_i_child_segments.at(0);
  const KDL::Segment& joint_i_child_segment = GetTreeElementSegment(segment_map.at(joint_i_child_segment_name));
  Eigen::Vector3d a_i = aerial_robot_model::kdlToEigen(seg_frames.at(joint_parent_link_names.at(joint_i)).M * joint_i_child_segment.getJoint().JointAxis());
  Eigen::Vector3d p_i = aerial_robot_model::kdlToEigen(seg_frames.at(joint_i_child_segment_name).p);

  Eigen::MatrixXd jacobian_i = Eigen::MatrixXd::Zero(6, full_body_ndof);

  if (std::find(joint_i_child_segments.begin(), joint_i_child_segments.end(), ref_frame) == joint_i_child_segments.end()) {
    return jacobian_i;
  }

  // joint part
  for(int j = 0; j < joint_num; j++)
    {
      std::string joint_j_name = joint_names.at(j);
      std::vector<std::string> joint_j_child_segments = joint_segment_map.at(joint_j_name);
      if (std::find(joint_j_child_segments.begin(), joint_j_child_segments.end(), ref_frame) == joint_j_child_segments.end()) {
        continue;
      }

      std::string joint_j_child_segment_name = joint_j_child_segments.at(0);
      const KDL::Segment& joint_j_child_segment = GetTreeElementSegment(segment_map.at(joint_j_child_segment_name));
      Eigen::Vector3d a_j = aerial_robot_model::kdlToEigen(seg_frames.at(joint_parent_link_names.at(j)).M * joint_j_child_segment.getJoint().JointAxis());
      Eigen::Vector3d p_j = aerial_robot_model::kdlToEigen(seg_frames.at(joint_j_child_segment_name).p);

      Eigen::Vector3d a_i_j(0,0,0);
      Eigen::Vector3d p_i_j(0,0,0);
      Eigen::Vector3d p_e_j = a_j.cross(p_e - p_j);
      if ( joint_hierachy.at(joint_j_name) <= joint_hierachy.at(joint_i_name)){
        // joint_j is close to root, or i = j

        // both i and j is revolute jont
        a_i_j = a_j.cross(a_i);
        p_i_j = a_j.cross(p_i - p_j);
      }

      jacobian_i.block(0, 6 + j, 3, 1) = root_rot * a_i_j.cross(p_e - p_i) + root_rot * a_i.cross(p_e_j - p_i_j); // force
      jacobian_i.block(3, 6 + j, 3, 1) = root_rot * a_i_j; // torque
    }

  // virtual 6dof root
  jacobian_i.block(0, 3, 3, 3) = - root_rot * aerial_robot_model::skew(a_i.cross(p_e - p_i));
  jacobian_i.block(3, 3, 3, 3) = - root_rot * aerial_robot_model::skew(a_i);
  return jacobian_i;
}

Eigen::MatrixXd HydrusXiFullyActuatedRobotModel::getSecondDerivativeRoot(std::string ref_frame, KDL::Vector offset)
{
  const int joint_num = getJointNum();
  const int full_body_ndof = 6 + joint_num;
  const auto& joint_parent_link_names = getJointParentLinkNames();
  const auto& segment_map = getTree().getSegments();
  const auto& seg_frames = getSegmentsTf();
  const auto root_rot = getCogDesireOrientation<Eigen::Matrix3d>();

  Eigen::Vector3d p_e = aerial_robot_model::kdlToEigen(seg_frames.at(ref_frame).p + seg_frames.at(ref_frame).M * offset);
  Eigen::MatrixXd out = Eigen::MatrixXd::Zero(3, full_body_ndof);

  // joint part
  for (int i = 0; i < joint_num; ++i) {

    std::string joint_name = getJointNames().at(i);
    std::vector<std::string> joint_child_segments = getJointSegmentMap().at(joint_name);

    if (std::find(joint_child_segments.begin(), joint_child_segments.end(), ref_frame) == joint_child_segments.end())
      {
        continue;
      }

    std::string joint_child_segment_name = joint_child_segments.at(0);
    const KDL::Segment& joint_child_segment = GetTreeElementSegment(segment_map.at(joint_child_segment_name));
    Eigen::Vector3d a = aerial_robot_model::kdlToEigen(seg_frames.at(joint_parent_link_names.at(i)).M * joint_child_segment.getJoint().JointAxis()); // fix bug
    Eigen::Vector3d p = aerial_robot_model::kdlToEigen(seg_frames.at(joint_child_segment_name).p);

    out.col(6 + i) = root_rot * a.cross(p_e - p);
  }

  // virtual root 6dof
  out.leftCols(3) = root_rot;
  out.middleCols(3,3) = - root_rot * aerial_robot_model::skew(p_e);

  return out;
}


inline Eigen::MatrixXd HydrusXiFullyActuatedRobotModel::convertJacobian(const Eigen::MatrixXd& in)
{
  const auto& joint_indices = getJointIndices();
  Eigen::MatrixXd out(6, getJointNum());

  int col_index = 0;
  for (const auto& joint_index : joint_indices) { // fix bug of order
    out.col(col_index) = in.col(joint_index);
    col_index++;
  }
  return out;
}

void HydrusXiFullyActuatedRobotModel::calcCOGJacobian()
{
  double mass_all = getMass();
  const auto cog_all = getCog<KDL::Frame>().p;
  const auto& segment_map = getTree().getSegments();
  const auto& seg_frames = getSegmentsTf();
  const auto& inertia_map = getInertiaMap();
  const auto& joint_names = getJointNames();
  const auto& joint_segment_map = getJointSegmentMap();

  int col_index = 0;
  /* fix bug: the joint_segment_map is reordered, which is not match the order of  joint_indeices_ or joint_names_ */
  // joint part
  for (const auto& joint_name : joint_names){
    std::string joint_child_segment_name = joint_segment_map.at(joint_name).at(0);
    KDL::Segment joint_child_segment = GetTreeElementSegment(segment_map.at(joint_child_segment_name));
    KDL::Vector a = seg_frames.at(joint_child_segment_name).M * joint_child_segment.getJoint().JointAxis();

    KDL::Vector r = seg_frames.at(joint_child_segment_name).p;
    KDL::RigidBodyInertia inertia = KDL::RigidBodyInertia::Zero();
    for (const auto& seg : joint_segment_map.at(joint_name)) {
      if (seg.find("thrust") == std::string::npos) {
        KDL::Frame f = seg_frames.at(seg);
        inertia = inertia + f * inertia_map.at(seg);
      }
    }
    KDL::Vector c = inertia.getCOG();
    double m = inertia.getMass();

    KDL::Vector cog_jacobian_col = a * (c - r) * m / mass_all;

    cog_jacobian_.col(6 + col_index) = aerial_robot_model::kdlToEigen(cog_jacobian_col);
    col_index++;
  }

  // virtual 6dof root
  cog_jacobian_.leftCols(3) = Eigen::MatrixXd::Identity(3,3);
  cog_jacobian_.middleCols(3, 3) = -aerial_robot_model::skew(aerial_robot_model::kdlToEigen(cog_all));
  cog_jacobian_ = getCogDesireOrientation<Eigen::Matrix3d>() * cog_jacobian_;
}

Eigen::MatrixXd HydrusXiFullyActuatedRobotModel::calcLambdaJacobain(const KDL::JntArray& joint_positions)
{
  // w.r.t root
  const auto& seg_frames = getSegmentsTf();
  const auto& inertia_map = getInertiaMap();
  const std::vector<Eigen::Vector3d> rotors_origin = getRotorsOriginFromCog<Eigen::Vector3d>();
  const std::vector<Eigen::Vector3d> rotors_normal = getRotorsNormalFromCog<Eigen::Vector3d>();
  const auto& rotor_direction = getRotorDirection();
  const int rotor_num = getRotorNum();
  const int joint_num = getJointNum();
  const int full_body_ndof = 6 + joint_num;
  const double m_f_rate = getMFRate();

  Eigen::VectorXd wrench_g = calcGravityWrench();
  Eigen::MatrixXd Q  = calcThrustJacobian();
  Eigen::VectorXd static_trhust_root = aerial_robot_model::pseudoinverse(Q) * (-wrench_g); // TODO: add external force

  ROS_DEBUG_STREAM("wrench gravity:" << wrench_g.transpose());
  ROS_DEBUG_STREAM("static thrust w.r.t. cog:" << calcStaticThrust().transpose());
  ROS_DEBUG_STREAM("static thrust w.r.t. root :" << static_trhust_root.transpose());

  /* derivative for gravity jacobian */
  Eigen::MatrixXd wrench_gravity_jacobian = Eigen::MatrixXd::Zero(6, full_body_ndof);
  for(const auto& inertia : inertia_map){
    wrench_gravity_jacobian.bottomRows(3) -= aerial_robot_model::skew(-inertia.second.getMass() * gravity_3d_) * getSecondDerivativeRoot(inertia.first, inertia.second.getCOG());
  }
  ROS_DEBUG_STREAM("wrench_gravity_jacobian w.r.t. root : \n" << wrench_gravity_jacobian);

  /* TODO: derivative for external force jacobian */

  /* derivative for thrust jacobian */
  Eigen::MatrixXd wrench_thrust_jacobian = Eigen::MatrixXd::Zero(6, full_body_ndof);
  for (int i = 0; i < rotor_num; ++i) {
    std::string thrust_name = std::string("thrust") + std::to_string(i + 1);
    Eigen::MatrixXd fr_wrench_unit_jacobian = Eigen::MatrixXd::Zero(6, full_body_ndof);
    Eigen::MatrixXd thrust_coord_jacobian = getJacobian(joint_positions, thrust_name);
    fr_wrench_unit_jacobian.topRows(3) = -skew(fr_wrench_units_.at(i).head(3)) * thrust_coord_jacobian.bottomRows(3);
    fr_wrench_unit_jacobian.bottomRows(3) = -skew(fr_wrench_units_.at(i).tail(3)) * thrust_coord_jacobian.bottomRows(3);

    wrench_thrust_jacobian.bottomRows(3) -= aerial_robot_model::skew(fr_wrench_units_.at(i).head(3)) * getSecondDerivativeRoot(thrust_name) * static_trhust_root(i);
    wrench_thrust_jacobian += fr_root_jacobians_.at(i).transpose() * fr_wrench_unit_jacobian * static_trhust_root(i);
  }

  ROS_DEBUG_STREAM("wrench_thrust_jacobian w.r.t. root : \n" << wrench_thrust_jacobian);

  Eigen::MatrixXd lambda_jacobian_root = aerial_robot_model::pseudoinverse(Q) * (- wrench_thrust_jacobian - wrench_gravity_jacobian);
  ROS_DEBUG_STREAM("wrench_jacobian w.r.t. root : \n" << wrench_thrust_jacobian + wrench_gravity_jacobian);
  ROS_DEBUG_STREAM("lambda_jacobian w.r.t. root : \n" << lambda_jacobian_root);

  return lambda_jacobian_root;
}

Eigen::MatrixXd HydrusXiFullyActuatedRobotModel::calcQMatrix()
{
  const std::vector<Eigen::Vector3d> rotors_origin = getRotorsOriginFromCog<Eigen::Vector3d>();
  const std::vector<Eigen::Vector3d> rotors_normal = getRotorsNormalFromCog<Eigen::Vector3d>();
  const auto& rotor_direction = getRotorDirection();
  const int rotor_num = getRotorNum();
  const double m_f_rate = getMFRate();

  //Q : WrenchAllocationMatrix
  Eigen::MatrixXd Q(6, rotor_num);
  for (unsigned int i = 0; i < rotor_num; ++i) {
    Q.block(0, i, 3, 1) = rotors_normal.at(i);
    Q.block(3, i, 3, 1) = rotors_origin.at(i).cross(rotors_normal.at(i)) + m_f_rate * rotor_direction.at(i + 1) * rotors_normal.at(i);
  }

  return Q;
}

Eigen::MatrixXd HydrusXiFullyActuatedRobotModel::calcQMatrix(const std::vector<Eigen::Vector3d>& u, const std::vector<Eigen::Vector3d>& p, const std::map<int, int>& sigma)
{
  const int rotor_num = getRotorNum();
  const double m_f_rate = getMFRate();

  Eigen::MatrixXd Q(6, rotor_num);
  for (unsigned int i = 0; i < rotor_num; ++i) {
    Q.block(0, i, 3, 1) = u.at(i);
    Q.block(3, i, 3, 1) = p.at(i).cross(u.at(i)) + m_f_rate * sigma.at(i + 1) * u.at(i);
  }

  return Q;
}

Eigen::MatrixXd HydrusXiFullyActuatedRobotModel::calcStaticThrust()
{
  auto Q_inv = aerial_robot_model::pseudoinverse(calcQMatrix());
  Eigen::MatrixXd static_thrust = Q_inv * getMass() * gravity_;
  return static_thrust;
}

Eigen::MatrixXd HydrusXiFullyActuatedRobotModel::calcStaticThrust(const Eigen::MatrixXd& q_inv)
{
  Eigen::MatrixXd static_thrust = q_inv * getMass() * gravity_;
  return static_thrust;
}


void HydrusXiFullyActuatedRobotModel::updateRobotModelImpl(const KDL::JntArray& joint_positions)
{
  aerial_robot_model::RobotModel::updateRobotModelImpl(joint_positions);
  updateJacobians(joint_positions);

  // return;

  const auto& seg_frames = getSegmentsTf();
  const auto& joint_index_map = getJointIndexMap();
  const auto& joint_segment_map = getJointSegmentMap();
  const auto& inertia_map = getInertiaMap();
  const int full_body_ndof = 6 + getJointNum();

  /* evaluate */
  double delta_angle = 0.00001; // [rad]
  Eigen::MatrixXd J_f = Eigen::MatrixXd::Zero(getRotorNum(), full_body_ndof);
  Eigen::MatrixXd J_t = Eigen::MatrixXd::Zero(getJointNum(), full_body_ndof);
  Eigen::MatrixXd J_g = Eigen::MatrixXd::Zero(6, full_body_ndof);
  Eigen::MatrixXd J_thrust = Eigen::MatrixXd::Zero(6, full_body_ndof);
  Eigen::MatrixXd J_thrust_j = Eigen::MatrixXd::Zero(6, full_body_ndof);
  Eigen::MatrixXd J_thrust_u = Eigen::MatrixXd::Zero(6, full_body_ndof);

  Eigen::MatrixXd lambda_jacobian = lambda_jacobian_;
  Eigen::MatrixXd joint_torque_jacobian = joint_torque_jacobian_;

  KDL::Rotation curr_root_att = getCogDesireOrientation<KDL::Rotation>();

  //lambda_jacobian = calcLambdaJacobain(joint_positions); // for debug, no need here

  Eigen::VectorXd nominal_static_thrust = static_thrust_;
  Eigen::VectorXd nominal_joint_torque = joint_torque_;
  Eigen::VectorXd nominal_wrench_g = calcGravityWrench();
  Eigen::VectorXd nominal_wrench_thrust = calcThrustJacobian() * nominal_static_thrust;
  std::vector<Eigen::MatrixXd> nominal_fr_root_jacobians = fr_root_jacobians_;
  std::vector<Eigen::VectorXd> nominal_fr_wrench_units = fr_wrench_units_;
  std::vector<Eigen::VectorXd> nominal_6dof_fr;
  for(int i = 0; i < getRotorNum(); i++)
    nominal_6dof_fr.push_back(nominal_fr_wrench_units.at(i) * nominal_static_thrust(i));

  int col_index = 6;
  const auto& joint_indices = getJointIndices();

#if 1
  // joint part 
  for (const auto& joint_index : joint_indices) {
      KDL::JntArray perturbation_joint_vector = joint_positions;
      perturbation_joint_vector(joint_index) += delta_angle;
      aerial_robot_model::RobotModel::updateRobotModelImpl(perturbation_joint_vector);
      Eigen::VectorXd perturbated_wrench_g = calcGravityWrench();
      J_g.col(col_index) = (perturbated_wrench_g - nominal_wrench_g) / delta_angle;
      Eigen::VectorXd perturbated_wrench_thrust = calcThrustJacobian() * nominal_static_thrust;
      J_thrust.col(col_index) = (perturbated_wrench_thrust - nominal_wrench_thrust) / delta_angle;

      perturbated_wrench_thrust = Eigen::VectorXd::Zero(6);
      for(int i = 0; i < getRotorNum(); i++)
        perturbated_wrench_thrust +=  fr_root_jacobians_.at(i).transpose() * nominal_6dof_fr.at(i);
      J_thrust_j.col(col_index) = (perturbated_wrench_thrust - nominal_wrench_thrust) / delta_angle;

      col_index++;
    }

  // virtual 6dof root
  auto virtualRootRotJacobianThrust = [&](int col)
    {
      aerial_robot_model::RobotModel::updateRobotModelImpl(joint_positions);
      Eigen::VectorXd perturbated_wrench_g = calcGravityWrench();
      J_g.col(col) = (perturbated_wrench_g - nominal_wrench_g) / delta_angle;
      Eigen::VectorXd perturbated_wrench_thrust = calcThrustJacobian() * nominal_static_thrust;
      J_thrust.col(col) = (perturbated_wrench_thrust - nominal_wrench_thrust) / delta_angle;
      perturbated_wrench_thrust = Eigen::VectorXd::Zero(6);
      for(int i = 0; i < getRotorNum(); i++)
        perturbated_wrench_thrust +=  fr_root_jacobians_.at(i).transpose() * nominal_6dof_fr.at(i);
      J_thrust_j.col(col) = (perturbated_wrench_thrust - nominal_wrench_thrust) / delta_angle;
    };

  // roll
  setCogDesireOrientation(curr_root_att * KDL::Rotation::RPY(delta_angle, 0, 0)); // set the orientation of root
  virtualRootRotJacobianThrust(3);

  // pitch
  setCogDesireOrientation(curr_root_att * KDL::Rotation::RPY(0, delta_angle, 0)); // set the orientation of root
  virtualRootRotJacobianThrust(4);

  // yaw
  setCogDesireOrientation(curr_root_att * KDL::Rotation::RPY(0, 0, delta_angle)); // set the orientation of root
  virtualRootRotJacobianThrust(5);

  // reset
  setCogDesireOrientation(curr_root_att);

  ROS_DEBUG_STREAM("numerical result of wrench_gravity_jacobian: \n" << J_g);
  ROS_DEBUG_STREAM("numerical result of wrench_thrust_jacobian: \n" << J_thrust);
  ROS_DEBUG_STREAM("numerical result of wrench_thrust_jacobian_j: \n" << J_thrust_j);

  // throw std::runtime_error("test");

  col_index = 6;
  for (const auto& joint_index : joint_indices) {
      KDL::JntArray perturbation_joint_vector = joint_positions;
      perturbation_joint_vector(joint_index) += delta_angle;
      aerial_robot_model::RobotModel::updateRobotModelImpl(perturbation_joint_vector);
      updateJacobians(perturbation_joint_vector);
      J_f.col(col_index) = (static_thrust_ - nominal_static_thrust) / delta_angle;
      J_t.col(col_index) = (joint_torque_ - nominal_joint_torque) / delta_angle;
      col_index++;
    }

  // virtual 6dof root
  auto virtualRootRotJacobianLambda = [&](int col)
    {
      aerial_robot_model::RobotModel::updateRobotModelImpl(joint_positions);
      updateJacobians(joint_positions);
      J_f.col(col) = (static_thrust_ - nominal_static_thrust) / delta_angle;
      J_t.col(col) = (joint_torque_ - nominal_joint_torque) / delta_angle;
    };

  // roll
  setCogDesireOrientation(curr_root_att * KDL::Rotation::RPY(delta_angle, 0, 0)); // set the orientation of root
  virtualRootRotJacobianLambda(3);

  // pitch
  setCogDesireOrientation(curr_root_att * KDL::Rotation::RPY(0, delta_angle, 0)); // set the orientation of root
  virtualRootRotJacobianLambda(4);

  // yaw
  setCogDesireOrientation(curr_root_att * KDL::Rotation::RPY(0, 0, delta_angle)); // set the orientation of root
  virtualRootRotJacobianLambda(5);

  // reset
  setCogDesireOrientation(curr_root_att); // set the orientation of root

  ROS_DEBUG_STREAM("lambda_jacobian: \n" << lambda_jacobian);
  ROS_DEBUG_STREAM("numerical result of lambda_jacobian: \n" << J_f);
  ROS_DEBUG_STREAM("diff of lambda jacobian: \n" << J_f - lambda_jacobian);

  if((J_f - lambda_jacobian).maxCoeff() > fabs((J_f - lambda_jacobian).minCoeff()))
    ROS_INFO_STREAM("max diff of labda jacobian: " << (J_f - lambda_jacobian).maxCoeff());
  else
    ROS_INFO_STREAM("max diff of labda jacobian: " << fabs((J_f - lambda_jacobian).minCoeff()));

  ROS_DEBUG_STREAM("joint_torque_jacobian: \n" << joint_torque_jacobian);
  ROS_DEBUG_STREAM("numerical result of joint_torque_jacobian: \n" << J_t);
  ROS_DEBUG_STREAM("diff of joint torque jacobian: \n" << J_t - joint_torque_jacobian);

  if((J_t - joint_torque_jacobian).maxCoeff() > fabs((J_t - joint_torque_jacobian).minCoeff()))
    ROS_INFO_STREAM("max diff of torque jacobian: " << (J_t - joint_torque_jacobian).maxCoeff());
  else
    ROS_INFO_STREAM("max diff of torque jacobian: " << fabs((J_t - joint_torque_jacobian).minCoeff()));

#endif

#if 0
  col_index = 6;
  delta_angle = 0.00001; // [rad]
  std::vector<Eigen::VectorXd> thrust_wrench = thrust_wrench_;
  std::vector<Eigen::MatrixXd> thrust_coord_jacobians = thrust_coord_jacobians_;
  std::vector<Eigen::MatrixXd> cog_coord_jacobians = cog_coord_jacobians_;
  Eigen::MatrixXd J_t_j = Eigen::MatrixXd::Zero(getJointNum(), full_body_ndof);

  auto virtualRootRotJacobianTorque = [&](int col)
    {
      aerial_robot_model::RobotModel::updateRobotModelImpl(joint_positions);
      updateJacobians(joint_positions);
#if 0
      for (int i = 0; i < getRotorNum(); ++i) {
        J_t_j.col(col)  -= (thrust_coord_jacobians_.at(i) - thrust_coord_jacobians.at(i)).rightCols(getJointNum()).transpose() * thrust_wrench.at(i)  / delta_angle;
      }
#else
      int seg_index = 0;
      for(const auto& inertia : inertia_map) {
        J_t_j.col(col)  -= (cog_coord_jacobians_.at(seg_index) - cog_coord_jacobians.at(seg_index)).rightCols(getJointNum()).transpose() * inertia.second.getMass() * (-gravity_)  / delta_angle;
        seg_index ++;
      }

#endif
    };

  for (const auto& joint_index : joint_indices) {
      KDL::JntArray perturbation_joint_vector = joint_positions;
      perturbation_joint_vector(joint_index) += delta_angle;
      aerial_robot_model::RobotModel::updateRobotModelImpl(perturbation_joint_vector);
      updateJacobians(perturbation_joint_vector);
#if 0
      for (int i = 0; i < getRotorNum(); ++i) {
        J_t_j.col(col_index)  -= (thrust_coord_jacobians_.at(i) - thrust_coord_jacobians.at(i)).rightCols(getJointNum()).transpose() * thrust_wrench.at(i)  / delta_angle;
      }
#else
      int seg_index = 0;
      for(const auto& inertia : inertia_map) {
        J_t_j.col(col_index)  -= (cog_coord_jacobians_.at(seg_index) - cog_coord_jacobians.at(seg_index)).rightCols(getJointNum()).transpose() * inertia.second.getMass() * (-gravity_)  / delta_angle;
        seg_index ++;
      }
#endif
      col_index++;
    }

  // roll
  setCogDesireOrientation(curr_root_att * KDL::Rotation::RPY(delta_angle, 0, 0)); // set the orientation of root
  virtualRootRotJacobianTorque(3);

  // pitch
  setCogDesireOrientation(curr_root_att * KDL::Rotation::RPY(0, delta_angle, 0)); // set the orientation of root
  virtualRootRotJacobianTorque(4);

  // yaw
  setCogDesireOrientation(curr_root_att * KDL::Rotation::RPY(0, 0, delta_angle)); // set the orientation of root
  virtualRootRotJacobianTorque(5);

  // reset
  setCogDesireOrientation(curr_root_att); // set the orientation of root


  ROS_INFO_STREAM("joint_torque_jacobian : \n" << joint_torque_jacobian);
  ROS_INFO_STREAM("numerical result of joint_torque_jacobian : \n" << J_t_j);
  ROS_INFO_STREAM("diff: \n" << J_t_j - joint_torque_jacobian);
#endif

  //throw std::runtime_error("test");
}


double HydrusXiFullyActuatedRobotModel::calcUTripleProduct(int i, int j, int k)
{
  const auto& u = getRotorsNormalFromCog<Eigen::Vector3d>();
  Eigen::Vector3d uixuj = u.at(i).cross(u.at(j));
  if (uixuj.norm() < 0.00001) {
    return 0.0;
  }
  return uixuj.dot(u.at(k)) / uixuj.norm();
}

inline double HydrusXiFullyActuatedRobotModel::calcTripleProduct(const Eigen::Vector3d& ui, const Eigen::Vector3d& uj, const Eigen::Vector3d& uk)
{
  Eigen::Vector3d uixuj = ui.cross(uj);
  if (uixuj.norm() < 0.00001) {
    return 0.0;
  }
  return uixuj.dot(uk) / uixuj.norm();
}

double HydrusXiFullyActuatedRobotModel::calcVTripleProduct(int i, int j, int k)
{
  const auto v = calcV();

  Eigen::Vector3d vixvj = v.at(i).cross(v.at(j));
  return vixvj.dot(v.at(k)) / vixvj.norm();
}

std::vector<Eigen::Vector3d> HydrusXiFullyActuatedRobotModel::calcV()
{
  const std::vector<Eigen::Vector3d> p = getRotorsOriginFromCog<Eigen::Vector3d>();
  const std::vector<Eigen::Vector3d> u = getRotorsNormalFromCog<Eigen::Vector3d>();
  const auto& sigma = getRotorDirection();
  const int rotor_num = getRotorNum();
  const double m_f_rate = getMFRate();
  std::vector<Eigen::Vector3d> v(rotor_num);

  for (int i = 0; i < rotor_num; ++i)
    v.at(i) = p.at(i).cross(u.at(i)) + m_f_rate * sigma.at(i + 1) * u.at(i);
  return v;
}

Eigen::VectorXd HydrusXiFullyActuatedRobotModel::calcFmin()
{
  const int rotor_num = getRotorNum();
  const double thrust_max = getThrustUpperLimit();
  Eigen::VectorXd f_min(rotor_num * (rotor_num - 1));

  const auto& u = getRotorsNormalFromCog<Eigen::Vector3d>();
  Eigen::Vector3d gravity_force = getMass() * gravity_3d_;

  int f_min_index = 0;
  for (int i = 0; i < rotor_num; ++i) {
    for (int j = 0; j < rotor_num; ++j) {
      if (i == j) continue;
      double f_min_ij = 0.0;
      for (int k = 0; k < rotor_num; ++k) {
        if (i == k || j == k) continue;
        double u_triple_product = calcUTripleProduct(i, j, k);
        f_min_ij += reluApprox(u_triple_product * thrust_max, epsilon_);
      }
      Eigen::Vector3d uixuj = u.at(i).cross(u.at(j));
      double d_fij = absApprox(f_min_ij - (uixuj.dot(gravity_force) / uixuj.norm()), epsilon_);
      f_min(f_min_index) = d_fij;
      f_min_index++;
    }
  }

  return f_min;
}

Eigen::VectorXd HydrusXiFullyActuatedRobotModel::calcTmin()
{
  const int rotor_num = getRotorNum();
  const double thrust_max = getThrustUpperLimit();
  Eigen::VectorXd t_min(rotor_num * (rotor_num - 1));

  const auto& v = calcV();
  int t_min_index = 0;

  for (int i = 0; i < rotor_num; ++i) {
    for (int j = 0; j < rotor_num; ++j) {
      if (i == j) continue;
      double t_min_ij = 0.0;
      for (int k = 0; k < rotor_num; ++k) {
        if (i == k || j == k) continue;
        double v_triple_product = calcVTripleProduct(i, j, k);
        t_min_ij += reluApprox(v_triple_product * thrust_max, epsilon_);
      }
      Eigen::Vector3d vixvj = v.at(i).cross(v.at(j));
      double d_tij = absApprox(t_min_ij, epsilon_);
      t_min(t_min_index) = d_tij;
      t_min_index++;
    }
  }

  return t_min;
}

Eigen::MatrixXd HydrusXiFullyActuatedRobotModel::calcWrenchAllocationMatrix()
{
  const std::vector<Eigen::Vector3d> rotors_origin = getRotorsOriginFromCog<Eigen::Vector3d>();
  const std::vector<Eigen::Vector3d> rotors_normal = getRotorsNormalFromCog<Eigen::Vector3d>();
  const int rotor_num = getRotorNum();

  //Q : WrenchAllocationMatrix
  Eigen::MatrixXd Q(6, rotor_num);
  double uav_mass_inv = 1.0 / getMass();
  Eigen::Matrix3d inertia_inv = getInertia<Eigen::Matrix3d>().inverse();
  for (unsigned int i = 0; i < rotor_num; ++i) {
    Q.block(0, i, 3, 1) = rotors_normal.at(i) * uav_mass_inv;
    Q.block(3, i, 3, 1) = inertia_inv * (rotors_origin.at(i).cross(rotors_normal.at(i)));
  }

  return Q;
}

Eigen::VectorXd HydrusXiFullyActuatedRobotModel::calcJointTorque(const sensor_msgs::JointState& joint_state)
{
  auto static_thrust = calcStaticThrust();
  KDL::JntArray joint_positions = jointMsgToKdl(joint_state);
  const auto& u = getRotorsNormalFromCog<Eigen::Vector3d>();
  const auto& sigma = getRotorDirection();
  const int joint_num = getJointNum();
  const int rotor_num = getRotorNum();
  const double m_f_rate = getMFRate();
  Eigen::VectorXd joint_torque = Eigen::VectorXd::Zero(joint_num);

  for (int i = 0; i < rotor_num; ++i) {
    std::string seg_name = std::string("thrust") + std::to_string(i + 1);
    Eigen::MatrixXd thrust_coord_jacobian = getJacobian(joint_positions, seg_name);
    Eigen::VectorXd wrench(6);
    Eigen::Vector3d thrust = u.at(i) * static_thrust(i);
    wrench.block(0, 0, 3, 1) = thrust;
    wrench.block(3, 0, 3, 1) = m_f_rate * sigma.at(i + 1) * thrust;
    joint_torque -= thrust_coord_jacobian.rightCols(joint_num).transpose() * wrench;
  }
  return joint_torque;
}

Eigen::VectorXd HydrusXiFullyActuatedRobotModel::calcGravityWrench()
{
  const auto& seg_frames = getSegmentsTf();
  const auto& inertia_map = getInertiaMap();
  const auto root_rot = getCogDesireOrientation<Eigen::Matrix3d>();

  Eigen::VectorXd wrench_g = Eigen::VectorXd::Zero(6);
  KDL::RigidBodyInertia link_inertia = KDL::RigidBodyInertia::Zero();
  std::vector<Eigen::MatrixXd> g_root_jacobians;
  for(const auto& inertia : inertia_map)
    {
      Eigen::MatrixXd jacobi_root = Eigen::MatrixXd::Identity(3, 6);
      Eigen::Vector3d p = root_rot * aerial_robot_model::kdlToEigen(seg_frames.at(inertia.first).p + seg_frames.at(inertia.first).M * inertia.second.getCOG());
      jacobi_root.rightCols(3) = - aerial_robot_model::skew(p);
      wrench_g += jacobi_root.transpose() *  inertia.second.getMass() * (-gravity_);
    }
  return wrench_g;
}

Eigen::MatrixXd HydrusXiFullyActuatedRobotModel::calcThrustJacobian()
{
  const auto& seg_frames = getSegmentsTf();
  const std::vector<Eigen::Vector3d> rotors_normal = getRotorsNormalFromCog<Eigen::Vector3d>();
  const auto& rotor_direction = getRotorDirection();
  const auto root_rot = getCogDesireOrientation<Eigen::Matrix3d>();
  const int rotor_num = getRotorNum();
  const int joint_num = getJointNum();
  const double m_f_rate = getMFRate();

  Eigen::MatrixXd Q(6, rotor_num);

  for (unsigned int i = 0; i < rotor_num; ++i) {
    std::string rotor = "thrust" + std::to_string(i + 1);
    Eigen::MatrixXd jacobi_root = Eigen::MatrixXd::Identity(6, 6);
    Eigen::Vector3d p = root_rot * aerial_robot_model::kdlToEigen(seg_frames.at(rotor).p);
    jacobi_root.topRightCorner(3,3) = - aerial_robot_model::skew(p);

    Eigen::VectorXd fr_wrench_unit = Eigen::VectorXd::Zero(6);
    fr_wrench_unit.head(3) = rotors_normal.at(i);
    fr_wrench_unit.tail(3) = m_f_rate * rotor_direction.at(i + 1) * rotors_normal.at(i);

    Q.col(i) = jacobi_root.transpose() * fr_wrench_unit;
    fr_root_jacobians_.at(i) = jacobi_root;
    fr_wrench_units_.at(i) = fr_wrench_unit;
  }

  return Q;
}
