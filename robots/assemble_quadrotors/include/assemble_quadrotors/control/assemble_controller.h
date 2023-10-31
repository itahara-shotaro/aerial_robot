// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, JSK Lab
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

#include <hydrus/hydrus_tilted_lqi_controller.h>
#include <aerial_robot_control/control/fully_actuated_controller.h>
#include <aerial_robot_control/control/fully_actuated_nobend_controller.h>
#include <assemble_quadrotors/model/assemble_robot_model.h>
#include <numeric>
namespace aerial_robot_control
{
  class AssembleController: public FullyActuatedController
  {

  public:
    AssembleController(){}
    virtual ~AssembleController() = default;

    void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                    boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                    double ctrl_loop_rate);

    boost::shared_ptr<AssembleTiltedRobotModel> assemble_robot_model_;

    bool update() override;

    double mass_trans_count_;
    double mass_trans_;
    double trans_rate_;
    double assemble_mass_;
    double dessemble_mass_;

  private:
    void sendCmd() override;
    void transMassCalc(double true_mass){
      mass_trans_= true_mass * (1.0-1.0/(mass_trans_count_ - 1.0/(trans_rate_ -1.0)));
    }
    double transThrustSumCalc(double now_sum, double pre_sum){
      double sum_ratio;
      double rate;
      rate = 1.0-6.0/(2.0*mass_trans_count_ + 6.0 );
      //ROS_INFO("rate is %f", rate);
      sum_ratio= (rate * now_sum + (1 - rate) * pre_sum)/ now_sum;
      return sum_ratio;
    }
    bool current_assemble_;
    double assemble_base_thrust_sum_;
    double dessemble_base_thrust_sum_;

    boost::shared_ptr<HydrusTiltedLQIController> dessemble_mode_controller_;
    boost::shared_ptr<FullyActuatedNobendController> assemble_mode_controller_;

    ros::Publisher desired_baselink_rot_pub_;

  protected:
    ros::NodeHandle assemble_nh_;
    ros::NodeHandle dessemble_nh_;
    std::string airframe_;

  };
};
