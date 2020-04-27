// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, JSK Lab
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

#include <aerial_robot_model/transformable_aerial_robot_model.h>
#include <algorithm>
#include <cmath>
#include <time.h>

using namespace aerial_robot_model;

class HydrusXiFullyActuatedRobotModel : public aerial_robot_model::RobotModel {
public:
  HydrusXiFullyActuatedRobotModel(bool init_with_rosparam,
                                  bool verbose = false,
                                  double epsilon = 10.0);
  virtual ~HydrusXiFullyActuatedRobotModel() = default;

  Eigen::MatrixXd calcWrenchAllocationMatrix();

  Eigen::MatrixXd getJacobian(const KDL::JntArray& joint_positions, std::string segment_name, KDL::Vector offset = KDL::Vector::Zero());
  inline Eigen::MatrixXd convertJacobian(const Eigen::MatrixXd& in);
  Eigen::MatrixXd getCOGJacobian() const {return cog_jacobian_;}
  std::vector<Eigen::MatrixXd> getUJacobian() const {return u_jacobian_;}
  std::vector<Eigen::MatrixXd> getVJacobian() const {return v_jacobian_;}
  std::vector<Eigen::MatrixXd> getPJacobian() const {return p_jacobian_;}
  std::vector<Eigen::MatrixXd> getQJacobian() const {return q_jacobian_;}
  Eigen::MatrixXd getLambdaJacobian() const {return lambda_jacobian_;}
  void updateJacobians(const KDL::JntArray& joint_positions);
  void updateJacobians(const sensor_msgs::JointState& joint_state);
  Eigen::MatrixXd calcQMatrix();
  Eigen::MatrixXd calcQMatrix(const std::vector<Eigen::Vector3d>& u, const std::vector<Eigen::Vector3d>& p, const std::map<int, int>& sigma);
  Eigen::MatrixXd calcStaticThrust();
  Eigen::MatrixXd calcStaticThrust(const Eigen::MatrixXd& q_inv);
  double calcUTripleProduct(int i, int j, int k);
  double calcVTripleProduct(int i, int j, int k);
  double calcTripleProduct(const Eigen::Vector3d& ui, const Eigen::Vector3d& uj, const Eigen::Vector3d& uk);
  Eigen::VectorXd getUTripleProductJacobian(int i, int j, int k) const { return u_triple_product_jacobian_.at(i).at(j).at(k); }
  Eigen::VectorXd getVTripleProductJacobian(int i, int j, int k) const { return v_triple_product_jacobian_.at(i).at(j).at(k); }
  Eigen::VectorXd calcFmin();
  Eigen::VectorXd calcTmin();
  Eigen::MatrixXd getFMinJacobian() const { return f_min_jacobian_; }
  Eigen::MatrixXd getTMinJacobian() const { return t_min_jacobian_; }
  Eigen::VectorXd getFminij() const { return f_min_ij_; }
  Eigen::VectorXd getTminij() const { return t_min_ij_; }
  Eigen::VectorXd getJointTorque() const {return joint_torque_;}
  Eigen::MatrixXd getJointTorqueJacobian() const {return joint_torque_jacobian_;}
  Eigen::VectorXd calcJointTorque(const sensor_msgs::JointState& joint_state);
  Eigen::MatrixXd getSecondDerivative(std::string ref_frame, int joint_i, KDL::Vector offset = KDL::Vector::Zero());
  Eigen::MatrixXd getSecondDerivativeRoot(std::string ref_frame, KDL::Vector offset = KDL::Vector::Zero());
  Eigen::VectorXd getHessian(std::string ref_frame, int joint_i, int joint_j, KDL::Vector offset = KDL::Vector::Zero());

  std::vector<Eigen::Vector3d> calcV();
  Eigen::VectorXd getStaticThrust() const {return static_thrust_;}

  KDL::JntArray convertMsgToKDL(const sensor_msgs::JointState& joint_state) {
    return jointMsgToKdl(joint_state);
  }

  KDL::JntArray convertEigenToKDL(const Eigen::VectorXd& joint_vector) {
    const auto& joint_indices = getJointIndices();
    KDL::JntArray joint_positions(getTree().getNrOfJoints());
    for (unsigned int i = 0; i < joint_indices.size(); ++i) {
      joint_positions(joint_indices.at(i)) = joint_vector(i);
    }
    return joint_positions;
  }

private:
  Eigen::VectorXd gravity_;
  Eigen::VectorXd gravity_3d_;
  double epsilon_;
  std::vector<Eigen::MatrixXd> u_jacobian_; //thrust direction vector index:rotor
  std::vector<Eigen::MatrixXd> v_jacobian_; //thrust torque direction vector index:rotor
  std::vector<Eigen::MatrixXd> p_jacobian_; //thrust position index:rotor
  std::vector<Eigen::MatrixXd> q_jacobian_; //allocation matrix index:rotor
  std::vector<Eigen::MatrixXd> thrust_coord_jacobians_;
  std::vector<Eigen::MatrixXd> cog_coord_jacobians_;
  Eigen::MatrixXd cog_jacobian_; //cog jacobian
  Eigen::MatrixXd lambda_jacobian_; //thrust force
  std::vector<std::vector<std::vector<Eigen::VectorXd> > > u_triple_product_jacobian_;
  std::vector<std::vector<std::vector<Eigen::VectorXd> > > v_triple_product_jacobian_;
  Eigen::VectorXd f_min_ij_;
  Eigen::VectorXd t_min_ij_;
  Eigen::MatrixXd f_min_jacobian_; //min force
  Eigen::MatrixXd t_min_jacobian_; //min torque
  Eigen::VectorXd joint_torque_;
  Eigen::MatrixXd joint_torque_jacobian_;
  Eigen::VectorXd static_thrust_;
  std::vector<Eigen::VectorXd> thrust_wrench_;

  void calcCOGJacobian();
  void getParamFromRos();

  Eigen::MatrixXd calcLambdaJacobain(const KDL::JntArray& joint_positions);
  void updateRobotModelImpl(const KDL::JntArray& joint_positions) override;

  Eigen::VectorXd calcGravityWrench();
  Eigen::MatrixXd calcThrustJacobian();

  std::vector<Eigen::MatrixXd> fr_root_jacobians_;
  std::vector<Eigen::VectorXd> fr_wrench_units_;

};
