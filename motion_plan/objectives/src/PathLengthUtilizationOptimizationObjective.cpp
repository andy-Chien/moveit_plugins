/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Rice University
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
*   * Neither the name of the Rice University nor the names of its
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

/* Author: Luis G. Torres, Jonathan Gammell */

#include "motion_plan/objectives/PathLengthUtilizationOptimizationObjective.h"
#include <memory>
#include <ompl/base/samplers/informed/PathLengthDirectInfSampler.h>

ompl::base::PathLengthUtilizationOptimizationObjective::PathLengthUtilizationOptimizationObjective(const SpaceInformationPtr &si)
  : ompl::base::OptimizationObjective(si)
{
    description_ = "Path Length & Utilization";

    // Setup a default cost-to-go heuristics:
    setCostToGoHeuristic(base::goalRegionCostToGo);
    setCostThreshold(infiniteCost());
}

ompl::base::Cost ompl::base::PathLengthUtilizationOptimizationObjective::stateCost(const State *) const
{
    return identityCost();
}

ompl::base::Cost ompl::base::PathLengthUtilizationOptimizationObjective::motionCost(const State *s1, const State *s2) const
{
    return Cost(si_->distance(s1, s2));
}

ompl::base::Cost ompl::base::PathLengthUtilizationOptimizationObjective::motionCost(const State *s1, const State *s2,
                                                                                    const float uti1, const float uti2) const
{
    float dis = si_->distance(s1, s2);
    return Cost(distance_weight_ * dis + (1 - distance_weight_) * dis * (1 - (uti1 + uti2) / 2));
}

ompl::base::Cost ompl::base::PathLengthUtilizationOptimizationObjective::motionCostHeuristic(const State *s1,
                                                                                  const State *s2) const
{
    return motionCost(s1, s2);
}

ompl::base::Cost ompl::base::PathLengthUtilizationOptimizationObjective::motionCostHeuristic(const State *s1, const State *s2, 
                                                                           const float uti1, const float uti2) const
{
    return motionCost(s1, s2, uti1, uti2);
}

void ompl::base::PathLengthUtilizationOptimizationObjective::setDistanceWeight(double dis_weight)
{
    this->distance_weight_ = dis_weight;
}

void ompl::base::PathLengthUtilizationOptimizationObjective::setMaxCost(double max_cost)
{
    this->max_cost_ = max_cost;
}

void ompl::base::PathLengthUtilizationOptimizationObjective::setEnableExploration(bool enable)
{
    this->enable_exploration_ = enable;
}

bool ompl::base::PathLengthUtilizationOptimizationObjective::getEnableExploration()
{
    return this->enable_exploration_;
}

bool ompl::base::PathLengthUtilizationOptimizationObjective::isAcceptable(const Cost cost) const
{
    return cost.value() < this->max_cost_;
}

ompl::base::InformedSamplerPtr ompl::base::PathLengthUtilizationOptimizationObjective::allocInformedStateSampler(
    const ProblemDefinitionPtr &probDefn, unsigned int maxNumberCalls) const
{
// Make the direct path-length informed sampler and return. If OMPL was compiled with Eigen, a direct version is
// available, if not a rejection-based technique can be used
    return std::make_shared<PathLengthDirectInfSampler>(probDefn, maxNumberCalls);
}
