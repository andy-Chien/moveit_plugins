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
#pragma once

#include <ompl/geometric/planners/prm/ConnectionStrategy.h>

namespace ompl
{
  namespace geometric
  {
    /**
    * \brief Return at most k neighbors, as long as they are also within a specified bound.
    */
    template <class Milestone>
    class KBoundedWeightStrategy : public KStrategy<Milestone>
    {
    public:
      /**
       * \brief Constructor
       *
       * \param k the maximum number of nearest neighbors to return
       * \param bound the maximum distance for any nearest neighbor to be returned
       * \param nn the nearest neighbors datastruture to use
       */

      KBoundedWeightStrategy(const unsigned int k, const double bound,
                const std::shared_ptr<NearestNeighbors<Milestone>> &nn, const std::vector<Milestone>& um)
        : KStrategy<Milestone>(k, nn), bound_(bound), usefulMilestone_(um)
      {
      }

      const auto &operator()(const Milestone &m)
      {
        auto &result = KStrategy<Milestone>::neighbors_;
        KStrategy<Milestone>::nn_->nearestK(m, KStrategy<Milestone>::k_, result);
        if (result.empty())
          return result;
        const auto &dist = KStrategy<Milestone>::nn_->getDistanceFunction();
        if (!KStrategy<Milestone>::nn_->reportsSortedResults())
        {
          std::sort(result.begin(), result.end(), 
            [&dist, &m](const Milestone a, const Milestone b) -> bool {
              return dist(a, m) < dist(b, m);
            }
          );
        }
        auto newCount = result.size();
        while (newCount > 0 && dist(result[newCount - 1], m) > bound_){
          --newCount;
        }
        result.resize(newCount);

        if (!usefulMilestone_.empty())
        {
          double nearestDist = std::numeric_limits<double>::max();
          auto nearest_it = usefulMilestone_.begin();
          for (auto it = usefulMilestone_.begin(); it != usefulMilestone_.end(); ++it) 
          {
            const double d = dist(*it, m);
            if (d < nearestDist){
              nearestDist = d;
              nearest_it = it;
            }
          }


          if (std::find(result.begin(), result.end(), *nearest_it) == result.end()){
            result.push_back(*nearest_it);
          }
        }
        return result;
      }

    protected:
      /** \brief The maximum distance at which nearby milestones are reported */
      const double bound_;
      const std::vector<Milestone>& usefulMilestone_;
    };
  }
}