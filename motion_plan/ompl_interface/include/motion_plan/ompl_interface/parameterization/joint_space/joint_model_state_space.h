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

#pragma once

#include "motion_plan/ompl_interface/parameterization/model_based_state_space.h"

namespace ompl_interface
{
class JointModelStateSpace : public ModelBasedStateSpace
{
public:
  static const std::string PARAMETERIZATION_TYPE;

  class StateType : public ModelBasedStateSpace::StateType
  {
  public:
    enum
    {
      POSE_COMPUTED = 32
    };

    StateType() : ModelBasedStateSpace::StateType()
    {
    }
    bool isPoseComputed() const
    {
      return flags & POSE_COMPUTED;
    }
    void markPoseComputed()
    {
      flags |= POSE_COMPUTED;
    }
    std::array<double, 3> pos{{0,0,0}};
    std::array<double, 4> quat{{1,0,0,0}};
  };
  JointModelStateSpace(const ModelBasedStateSpaceSpecification& spec);
  ~JointModelStateSpace() override;

  ompl::base::State* allocState() const override;

  void freeState(ompl::base::State* state) const override;

  bool computeStateFK(ompl::base::State* state) const;

  void interpolate(const ompl::base::State* from, const ompl::base::State* to, const double t,
                   ompl::base::State* state) const override;

  double distance(const ompl::base::State* state1, const ompl::base::State* state2) const override;

  ompl::base::StateSamplerPtr allocDefaultStateSampler() const override;

  void copyState(ompl::base::State* destination, const ompl::base::State* source) const override;

  void copyToOMPLState(ompl::base::State* state, const moveit::core::RobotState& rstate) const override;

  const std::string& getParameterizationType() const override
  {
    return PARAMETERIZATION_TYPE;
  }
private:
  void setKinematics(const moveit::core::JointModelGroup::KinematicsSolver& k);
  bool computeStateFK(StateType* full_state) const;
  const moveit::core::JointModelGroup* group_;
  kinematics::KinematicsBasePtr k_solver_;
  std::vector<size_t> bijection_;
  std::vector<std::string> fk_link_;
};
}  // namespace ompl_interface
