/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, Kentaro Wada.
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

/* Author: Kentaro Wada */

#include "moveit_capability/execute_trajectory_action_capability.h"

#include <moveit/trajectory_processing/trajectory_tools.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/move_group/capability_names.h>
#include <moveit/robot_state/conversions.h>

namespace move_group
{

moveit_controller_manager::ExecutionStatus MoveGroupExecuteTrajectoryActionT::execute_status_ = moveit_controller_manager::ExecutionStatus::UNKNOWN;

MoveGroupExecuteTrajectoryActionT::MoveGroupExecuteTrajectoryActionT() : MoveGroupCapability("ExecuteTrajectoryAction")
{
}


void MoveGroupExecuteTrajectoryActionT::initialize()
{
  private_nh_ = ros::NodeHandle();
  // start the move action server
  execute_action_server_.reset(new actionlib::SimpleActionServer<moveit_msgs::ExecuteTrajectoryAction>(
      root_node_handle_, EXECUTE_ACTION_NAME,
      boost::bind(&MoveGroupExecuteTrajectoryActionT::executePathCallback, this, _1), false));
  execute_action_server_->registerPreemptCallback(
      boost::bind(&MoveGroupExecuteTrajectoryActionT::preemptExecuteTrajectoryCallback, this));
  execute_action_server_->start();
  collision_robotstate_publisher_ = private_nh_.advertise<moveit_msgs::DisplayRobotState>("collision_robotstate", 1, true);
}

void MoveGroupExecuteTrajectoryActionT::executeCallback(const moveit_controller_manager::ExecutionStatus& status)
{ 
  MoveGroupExecuteTrajectoryActionT::execute_status_ = status; 
}

void MoveGroupExecuteTrajectoryActionT::executePathCallback(const moveit_msgs::ExecuteTrajectoryGoalConstPtr& goal)
{
  moveit_msgs::ExecuteTrajectoryResult action_res;
  if (!context_->trajectory_execution_manager_)
  {
    const std::string response = "Cannot execute trajectory since ~allow_trajectory_execution was set to false";
    action_res.error_code.val = moveit_msgs::MoveItErrorCodes::CONTROL_FAILED;
    execute_action_server_->setAborted(action_res, response);
    return;
  }

  executePath(goal, action_res);

  const std::string response = getActionResultString(action_res.error_code, false, false);
  if (action_res.error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
  {
    execute_action_server_->setSucceeded(action_res, response);
  }
  else if (action_res.error_code.val == moveit_msgs::MoveItErrorCodes::PREEMPTED)
  {
    execute_action_server_->setPreempted(action_res, response);
  }
  else
  {
    execute_action_server_->setAborted(action_res, response);
  }

  setExecuteTrajectoryState(IDLE);
}

void MoveGroupExecuteTrajectoryActionT::executePath(const moveit_msgs::ExecuteTrajectoryGoalConstPtr& goal,
                                                   moveit_msgs::ExecuteTrajectoryResult& action_res)
{
  ROS_INFO("executePath, executePath, executePath, executePath, executePath");
  ROS_INFO_NAMED(getName(), "Execution request received");

  context_->trajectory_execution_manager_->clear();
  if (context_->trajectory_execution_manager_->push(goal->trajectory))
  {
    setExecuteTrajectoryState(MONITOR);
    MoveGroupExecuteTrajectoryActionT::execute_status_ = moveit_controller_manager::ExecutionStatus::RUNNING;
    trajectory_execution_manager::TrajectoryExecutionManager::ExecutionCompleteCallback execution_callback;
    execution_callback = MoveGroupExecuteTrajectoryActionT::executeCallback;
    context_->trajectory_execution_manager_->execute(execution_callback);

    planning_scene::PlanningSceneConstPtr planning_scene = context_->planning_scene_monitor_->getPlanningScene();
    robot_trajectory::RobotTrajectory t(planning_scene->getRobotModel(), "");
    t.setRobotTrajectoryMsg(planning_scene->getCurrentState(), goal->trajectory);
    ros::Rate r(30);
    std::size_t wpc = t.getWayPointCount();
    std::pair<int, int> path_segment;
    ros::Time start_time;
    try
    {
      while(node_handle_.ok() && execute_status_ == moveit_controller_manager::ExecutionStatus::RUNNING)
      {
        r.sleep();
        planning_scene_monitor::LockedPlanningSceneRO lscene(context_->planning_scene_monitor_);

        path_segment = context_->trajectory_execution_manager_->getCurrentExpectedTrajectoryIndex();
        if(path_segment.second == -1)
          break;
        std::size_t way_point_indx = std::min(std::size_t(path_segment.second), wpc - 1);

        if(!checkWayPointCollision(way_point_indx, planning_scene, t))
        {
          ROS_WARN("!!!!!!Collision detected during execution!!!!!!");
          context_->trajectory_execution_manager_->stopExecution();
          execute_status_ = moveit_controller_manager::ExecutionStatus::ABORTED;
          break;
        }
      }
    }
    catch (...)
    {
      ROS_ERROR("!!!!!!!Collision detected during execution Failed!!!!!!!");
    }
    if(execute_status_ == moveit_controller_manager::ExecutionStatus::RUNNING)
    {
      execute_status_ = context_->trajectory_execution_manager_->waitForExecution();
    }
    if (execute_status_ == moveit_controller_manager::ExecutionStatus::SUCCEEDED)
    {
      action_res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
    }
    else if (execute_status_ == moveit_controller_manager::ExecutionStatus::PREEMPTED)
    {
      action_res.error_code.val = moveit_msgs::MoveItErrorCodes::PREEMPTED;
    }
    else if (execute_status_ == moveit_controller_manager::ExecutionStatus::TIMED_OUT)
    {
      action_res.error_code.val = moveit_msgs::MoveItErrorCodes::TIMED_OUT;
    }
    else if (execute_status_ == moveit_controller_manager::ExecutionStatus::ABORTED)
    {
      action_res.error_code.val = moveit_msgs::MoveItErrorCodes::MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE;
    }
    else
    {
      action_res.error_code.val = moveit_msgs::MoveItErrorCodes::CONTROL_FAILED;
    }
    ROS_INFO_STREAM_NAMED(getName(), "Execution completed: " << execute_status_.asString());
  }
  else
  {
    action_res.error_code.val = moveit_msgs::MoveItErrorCodes::CONTROL_FAILED;
  }
}

bool MoveGroupExecuteTrajectoryActionT::checkWayPointCollision(std::size_t way_point_indx, planning_scene::PlanningSceneConstPtr planning_scene, robot_trajectory::RobotTrajectory& t)
{
  std::size_t wpc = t.getWayPointCount();
  robot_state::RobotState start_state(planning_scene->getCurrentState());
  robot_state::RobotState collision_detect_state(start_state);
  robot_state::RobotState next_waypoint_state(t.getWayPoint(way_point_indx));
  collision_detection::CollisionRequest req;
  collision_detection::CollisionResult res;
  double state_distance = start_state.distance(next_waypoint_state);
  double max_detect_ang = 0;
  double tmp_detect_ang = 0;
  double interpolate_time = 0;
  req.contacts = true;

  for(std::size_t joints_indx=0; joints_indx<std::min(start_state.getVariableCount(), std::size_t(3)); joints_indx++)
    max_detect_ang += start_state.getVariableVelocity(joints_indx);
  max_detect_ang = std::min(std::max(max_detect_ang / 1.5, M_PI / 6), M_PI / 1.5);

  for(double detect_ang = 0; detect_ang < max_detect_ang; detect_ang += M_PI / 60)
  {
    interpolate_time = ((detect_ang - tmp_detect_ang)<= state_distance && state_distance > 0.1) ? (detect_ang - tmp_detect_ang) / state_distance : 1;
    start_state.interpolate(next_waypoint_state, interpolate_time, collision_detect_state);
    planning_scene->checkCollision(req, res, collision_detect_state);
    
    if (res.collision)
    {
      for(auto it = res.contacts.begin(); it != res.contacts.end(); it++)
      {
        if(!it->first.first.compare("<octomap>") || !it->first.second.compare("<octomap>"))
        {
          std::cout<<it->first.first<<", "<<it->first.second<<std::endl;
          return false;
        }
      }
    }
    
    if(detect_ang >= state_distance)
    {
      if(way_point_indx >= wpc - 1)
        break;
      tmp_detect_ang = detect_ang;
      way_point_indx = std::min(way_point_indx + 1, wpc - 1);
      start_state = next_waypoint_state;
      next_waypoint_state = t.getWayPoint(way_point_indx);
      state_distance = start_state.distance(next_waypoint_state);
    }
  }
  return true;
}

void MoveGroupExecuteTrajectoryActionT::preemptExecuteTrajectoryCallback()
{
  context_->trajectory_execution_manager_->stopExecution(true);
}

void MoveGroupExecuteTrajectoryActionT::setExecuteTrajectoryState(MoveGroupState state)
{
  moveit_msgs::ExecuteTrajectoryFeedback execute_feedback;
  execute_feedback.state = stateToStr(state);
  execute_action_server_->publishFeedback(execute_feedback);
}

}  // namespace move_group

#include <class_loader/class_loader.hpp>
CLASS_LOADER_REGISTER_CLASS(move_group::MoveGroupExecuteTrajectoryActionT, move_group::MoveGroupCapability)
