/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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

/* Author: Andy Chien */

#include <moveit/ompl_interface/model_based_planning_context.h>
#include <ompl/base/spaces/constraint/ConstrainedStateSpace.h>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

#include "motion_plan/ompl_interface/detail/motion_validity_checker.h"


namespace ompl_interface
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit.ompl_planning.motion_validity_checker");

MotionValidityChecker::MotionValidityChecker(const ModelBasedPlanningContext* pc)
  : ompl::base::MotionValidator(pc->getOMPLSimpleSetup()->getSpaceInformation())
  , planning_context_(pc)
  , tss_(pc->getCompleteInitialRobotState())
  , verbose_(false)
{
  collision_request_with_distance_.distance = true;
  collision_request_with_cost_.cost = true;

  collision_request_simple_.group_name = planning_context_->getGroupName();
  collision_request_with_distance_.group_name = planning_context_->getGroupName();
  collision_request_with_cost_.group_name = planning_context_->getGroupName();

  collision_request_simple_verbose_ = collision_request_simple_;
  collision_request_simple_verbose_.verbose = true;

  collision_request_with_distance_verbose_ = collision_request_with_distance_;
  collision_request_with_distance_verbose_.verbose = true;
}

void MotionValidityChecker::setVerbose(bool flag)
{
  verbose_ = flag;
}

bool MotionValidityChecker::checkMotion(
  const ompl::base::State *s1, const ompl::base::State *s2) const
{
  assert(s1 != nullptr && s2 != nullptr);

  moveit::core::RobotState* robot_state = tss_.getStateStorage();
  planning_context_->getOMPLStateSpace()->copyToRobotState(*robot_state, s1);
  // check collision avoidance
  collision_detection::CollisionResult res;
  planning_context_->getPlanningScene()->checkCollision(
      verbose_ ? collision_request_simple_verbose_ : collision_request_simple_, res, *robot_state);
  if (res.collision){
    return false;
  }
  planning_context_->getOMPLStateSpace()->copyToRobotState(*robot_state, s2);
  planning_context_->getPlanningScene()->checkCollision(
      verbose_ ? collision_request_simple_verbose_ : collision_request_simple_, res, *robot_state);
  return !res.collision;
}

bool MotionValidityChecker::checkMotion(
  const ompl::base::State *s1, const ompl::base::State *s2,
  std::pair<ompl::base::State *, double> &lastValid) const
{
  return true;
}

// bool MotionValidityChecker::isValid(const ompl::base::State* state, bool verbose) const
// {
//   assert(state != nullptr);
//   // check collision avoidance
//   collision_detection::CollisionResult res;
//   planning_context_->getPlanningScene()->checkCollision(
//       verbose ? collision_request_simple_verbose_ : collision_request_simple_, res, *robot_state);
//   if (!res.collision)
//   {
//     const_cast<ob::State*>(state)->as<ModelBasedStateSpace::StateType>()->markValid();
//   }
//   else
//   {
//     const_cast<ob::State*>(state)->as<ModelBasedStateSpace::StateType>()->markInvalid();
//   }
//   return !res.collision;
// }
}  // namespace ompl_interface
