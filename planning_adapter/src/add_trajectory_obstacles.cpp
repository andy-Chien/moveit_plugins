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

#include <moveit/planning_request_adapter/planning_request_adapter.h>
#include <class_loader/class_loader.hpp>

#include "mr_msgs/srv/get_robot_trajectory_obstacle.hpp"

namespace planning_adapter
{
class AddTrajectoryObstacles : public planning_request_adapter::PlanningRequestAdapter
{
public:
  std::string getDescription() const override
  {
    return "No Op";
  }

  bool adaptAndPlan(const PlannerFn& planner, const planning_scene::PlanningSceneConstPtr& planning_scene,
                    const planning_interface::MotionPlanRequest& req, planning_interface::MotionPlanResponse& res,
                    std::vector<std::size_t>& /*added_path_index*/) const override
  {
    auto obs_req = std::make_shared<mr_msgs::srv::GetRobotTrajectoryObstacle::Request>();
    return planner(planning_scene, req, res);
  }

  void initialize(const rclcpp::Node::SharedPtr& node, const std::string& /* parameter_namespace */) override
  {
    
    const auto& name_without_slash = [&](const char* ns){
      const auto& name = std::string(ns);
      if(name.size() > 1 && name.at(0) == '/'){
        return std::string(name.begin() + 1, name.end());
      }else{
        return name;
      }
    };

    robot_name_ = name_without_slash(node->get_namespace());
    client_ = node->create_client<
      mr_msgs::srv::GetRobotTrajectoryObstacle>("/get_trajectory_obstacle");
    RCLCPP_INFO(node->get_logger(), 
      "AddTrajectoryObstacles planning adapter for robot '%s' is loaded", robot_name_.c_str());
  }
protected:
  std::string robot_name_;
  rclcpp::Client<mr_msgs::srv::GetRobotTrajectoryObstacle>::SharedPtr client_;
};
}  // namespace planning_adapter

CLASS_LOADER_REGISTER_CLASS(planning_adapter::AddTrajectoryObstacles, planning_request_adapter::PlanningRequestAdapter)
