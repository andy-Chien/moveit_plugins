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

/* Author: Andy Chien  */

#pragma once

#include <moveit/ompl_interface/detail/threadsafe_state_storage.h>
#include <moveit/collision_detection/collision_common.h>
#include <ompl/base/MotionValidator.h>

namespace ompl_interface
{
class ModelBasedPlanningContext;

/** @class MotionValidityChecker
    @brief An interface for a OMPL state validity checker*/
class MotionValidityChecker : public ompl::base::MotionValidator
{
public:
  MotionValidityChecker(const ModelBasedPlanningContext* planning_context);

  /** \brief Check if the path between two states (from \e s1 to \e s2) is valid. This function assumes \e s1
   is valid.

    \note This function updates the number of valid and invalid segments. */
  bool checkMotion(const ompl::base::State *s1, const ompl::base::State *s2) const override;

  /** \brief Check if the path between two states is valid. Also compute the last state that was
    valid and the time of that state. The time is used to parametrize the motion from \e s1 to \e s2, \e s1
    being at t =
    0 and \e s2 being at t = 1. This function assumes \e s1 is valid.
    \param s1 start state of the motion to be checked (assumed to be valid)
    \param s2 final state of the motion to be checked
    \param lastValid first: storage for the last valid state (may be nullptr, if the user does not care
    about the exact state); this need not be different from \e s1 or \e s2. second: the time (between 0 and
    1) of the last valid state, on the motion from \e s1 to \e s2. If the function returns false, \e
    lastValid.first must be set to a valid state, even if that implies copying \e s1 to \e lastValid.first
    (in case \e lastValid.second = 0). If the function returns true, \e lastValid.first and \e
    lastValid.second should \b not be modified.

    \note This function updates the number of valid and invalid segments. */
  bool checkMotion(
    const ompl::base::State *s1, const ompl::base::State *s2, 
    std::pair<ompl::base::State *, double> &lastValid) const override;

  void setVerbose(bool flag);

protected:
  const ModelBasedPlanningContext* planning_context_;
  TSStateStorage tss_;
  collision_detection::CollisionRequest collision_request_simple_;
  collision_detection::CollisionRequest collision_request_with_distance_;
  collision_detection::CollisionRequest collision_request_simple_verbose_;
  collision_detection::CollisionRequest collision_request_with_distance_verbose_;

  collision_detection::CollisionRequest collision_request_with_cost_;
  bool verbose_;
};
}  // namespace ompl_interface
