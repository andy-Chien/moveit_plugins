/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
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

/* Author: Ioan Sucan */

#include "motion_plan/ompl_interface/parameterization/joint_space/joint_model_state_space.h"

const std::string ompl_interface::JointModelStateSpace::PARAMETERIZATION_TYPE = "JointModel";

namespace ompl_interface
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("joint_model_state_space");
} // namespace ompl_interface
ompl_interface::JointModelStateSpace::JointModelStateSpace(const ModelBasedStateSpaceSpecification& spec)
  : ModelBasedStateSpace(spec), group_(spec_.joint_model_group_)
{
  setName(getName() + "_" + PARAMETERIZATION_TYPE);
  if (spec_.joint_model_group_->getGroupKinematics().first){
    setKinematics(spec.joint_model_group_->getGroupKinematics().first);
  }
  else if (!spec_.joint_model_group_->getGroupKinematics().second.empty()){
    const auto& m = spec.joint_model_group_->getGroupKinematics().second;
    if (m.size() > 1)
      RCLCPP_ERROR(LOGGER, "Not support for multi group yet");
    const auto& it = m.begin();
    setKinematics((*it).second);
  }
}

ompl_interface::JointModelStateSpace::~JointModelStateSpace() = default;

ompl::base::State* ompl_interface::JointModelStateSpace::allocState() const
{
  auto* state = new StateType();
  state->values =
      new double[variable_count_];  // need to allocate this here since ModelBasedStateSpace::allocState() is not called
  return state;
}

void ompl_interface::JointModelStateSpace::freeState(ompl::base::State* state) const
{
  ModelBasedStateSpace::freeState(state);
}

void ompl_interface::JointModelStateSpace::setKinematics(
    const moveit::core::JointModelGroup::KinematicsSolver& k)
{
  bijection_ = k.bijection_;
  k_solver_ = k.allocator_(group_);
  fk_link_.resize(1, k_solver_->getTipFrame());
  if (!fk_link_[0].empty() && fk_link_[0][0] == '/')
    fk_link_[0] = fk_link_[0].substr(1);
}

bool ompl_interface::JointModelStateSpace::computeStateFK(StateType* full_state) const
{
  // read the values from the joint state, in the order expected by the kinematics solver
  std::vector<double> values(bijection_.size());
  for (unsigned int i = 0; i < bijection_.size(); ++i)
    values[i] = full_state->values[bijection_[i]];

  // compute forward kinematics for the link of interest
  std::vector<geometry_msgs::msg::Pose> poses;
  if (!k_solver_->getPositionFK(fk_link_, values, poses))
    return false;

  // copy the resulting data to the desired location in the state
  auto& pos = full_state->pos;
  auto& quat = full_state->quat;
  pos[0] = poses[0].position.x;
  pos[1] = poses[0].position.y;
  pos[2] = poses[0].position.z;
  quat[0] = poses[0].orientation.w;
  quat[1] = poses[0].orientation.x;
  quat[2] = poses[0].orientation.y;
  quat[3] = poses[0].orientation.z;
  return true;
}

bool ompl_interface::JointModelStateSpace::computeStateFK(ompl::base::State* state) const
{
  if (state->as<StateType>()->isPoseComputed()){
    return true;
  }
  if (!computeStateFK(state->as<StateType>()))
  {
    state->as<StateType>()->markInvalid();
    RCLCPP_ERROR(LOGGER, "State FK Compoute Failed!");
    return false;
  }
  state->as<StateType>()->markPoseComputed();
  return true;
}

void ompl_interface::JointModelStateSpace::copyState(ompl::base::State* destination,
                                                    const ompl::base::State* source) const
{
  // copy the state data
  ModelBasedStateSpace::copyState(destination, source);
  memcpy(destination->as<StateType>()->pos.data(),
    source->as<StateType>()->pos.data(), 3 * sizeof(double));
  memcpy(destination->as<StateType>()->quat.data(),
    source->as<StateType>()->quat.data(), 4 * sizeof(double));
}

void ompl_interface::JointModelStateSpace::copyToOMPLState(ompl::base::State* state,
                                                          const moveit::core::RobotState& rstate) const
{
  ModelBasedStateSpace::copyToOMPLState(state, rstate);
  computeStateFK(state);
}

void ompl_interface::JointModelStateSpace::interpolate(const ompl::base::State* from, const ompl::base::State* to,
                                                       const double t, ompl::base::State* state) const
{
  ModelBasedStateSpace::interpolate(from, to, t, state);
  computeStateFK(state);
}

double ompl_interface::JointModelStateSpace::distance(const ompl::base::State* state1,
                                                      const ompl::base::State* state2) const
{
  if (!state1->as<StateType>()->isPoseComputed() || !state2->as<StateType>()->isPoseComputed()){
    RCLCPP_ERROR(LOGGER, "State FK Not Computed!");
    return group_->distance(state1->as<StateType>()->values, state2->as<StateType>()->values);
  }
  const auto& pos_dis = [](const std::array<double, 3>& a, const std::array<double, 3>& b){
    return std::hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
  };
  const auto& quat_dis = [](const std::array<double, 4>& a, const std::array<double, 4>& b){
    const std::array<double, 4> d = {a[0] - b[0], a[1] - b[1], a[2] - b[2], a[3] - b[3]};
    const double qd = sqrt(d[0]*d[0] + d[1]*d[1] + d[2]*d[2] + d[3]*d[3]);
    return (qd > 1) ? 2 - qd : qd;
  };
  const double jl = group_->distance(state1->as<StateType>()->values, state2->as<StateType>()->values);
  const double pl = pos_dis(state1->as<StateType>()->pos, state2->as<StateType>()->pos);
  const double ql = quat_dis(state1->as<StateType>()->quat, state2->as<StateType>()->quat);
  return 0.2 * jl + 0.4 * pl + 0.4 * ql;
}

ompl::base::StateSamplerPtr ompl_interface::JointModelStateSpace::allocDefaultStateSampler() const
{
  class JointModelStateSampler : public ompl::base::StateSampler
  {
  public:
    JointModelStateSampler(const ompl::base::StateSpace* space, ompl::base::StateSamplerPtr sampler)
      : ompl::base::StateSampler(space), sampler_(std::move(sampler))
    {
    }

    void sampleUniform(ompl::base::State* state) override
    {
      sampler_->sampleUniform(state);
      space_->as<JointModelStateSpace>()->computeStateFK(state);
    }

    void sampleUniformNear(ompl::base::State* state, const ompl::base::State* near, const double distance) override
    {
      sampler_->sampleUniformNear(state, near, distance);
      space_->as<JointModelStateSpace>()->computeStateFK(state);
    }

    void sampleGaussian(ompl::base::State* state, const ompl::base::State* mean, const double stdDev) override
    {
      sampler_->sampleGaussian(state, mean, stdDev);
      space_->as<JointModelStateSpace>()->computeStateFK(state);
    }
    ompl::base::StateSamplerPtr sampler_;
  };

  return ompl::base::StateSamplerPtr(static_cast<ompl::base::StateSampler*>(
      new JointModelStateSampler(this, ModelBasedStateSpace::allocDefaultStateSampler())));
}