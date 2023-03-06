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

/* Author: Kentaro Wada, Andy Chien*/

#include "moveit_capability/async_execute_trajectory_action_capability.h"

#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/plan_execution/plan_execution.h>
#include <moveit/trajectory_processing/trajectory_tools.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/move_group/capability_names.h>

namespace move_group
{
static const rclcpp::Logger LOGGER =
    rclcpp::get_logger("async_execute_trajectory_action_capability");

MoveGroupAsyncExecuteTrajectoryAction::MoveGroupAsyncExecuteTrajectoryAction()
 : MoveGroupCapability("AsyncExecuteTrajectoryAction")
{
}

void MoveGroupAsyncExecuteTrajectoryAction::initialize()
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  auto node = context_->moveit_cpp_->getNode();
  cb_group_ = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  // start the move action server
  execute_action_server_ = rclcpp_action::create_server<ExecTrajectory>(
      node->get_node_base_interface(), node->get_node_clock_interface(), node->get_node_logging_interface(),
      node->get_node_waitables_interface(), EXECUTE_ACTION_NAME,
      [](const rclcpp_action::GoalUUID& /*unused*/, std::shared_ptr<const ExecTrajectory::Goal> /*unused*/) {
        RCLCPP_INFO(LOGGER, "Received goal request");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](const std::shared_ptr<ExecTrajectoryGoal>& /* unused */) {
        RCLCPP_INFO(LOGGER, "Received request to cancel goal");
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const auto& goal) { executePathCallback(goal); },
      rcl_action_server_get_default_options(), cb_group_);

  RCLCPP_INFO(LOGGER, "========== MoveGroupAsyncExecuteTrajectoryAction is initialized ==========");
}

void MoveGroupAsyncExecuteTrajectoryAction::executePathCallback(std::shared_ptr<ExecTrajectoryGoal> goal)
{
  auto action_res = std::make_shared<ExecTrajectory::Result>();
  if (!context_->trajectory_execution_manager_)
  {
    const std::string response = "Cannot execute trajectory since ~allow_trajectory_execution was set to false";
    action_res->error_code.val = moveit_msgs::msg::MoveItErrorCodes::CONTROL_FAILED;
    goal->abort(action_res);
    return;
  }

  executePath(goal, action_res);

  const std::string response = getActionResultString(action_res->error_code, false, false);
  auto fb = std::make_shared<ExecTrajectory::Feedback>();
  fb->state = response;
  if (action_res->error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    goal->publish_feedback(fb);
    goal->succeed(action_res);
  }
  else
  {
    goal->publish_feedback(fb);
    goal->abort(action_res);
  }

  setExecuteTrajectoryState(IDLE, goal);
}

void MoveGroupAsyncExecuteTrajectoryAction::executePath(const std::shared_ptr<ExecTrajectoryGoal>& goal,
                                                   std::shared_ptr<ExecTrajectory::Result>& action_res)
{
  RCLCPP_INFO(LOGGER, "Execution request received");

  context_->trajectory_execution_manager_->clear();
  if (context_->trajectory_execution_manager_->push(goal->get_goal()->trajectory))
  {
    setExecuteTrajectoryState(MONITOR, goal);
    context_->trajectory_execution_manager_->execute();
    moveit_controller_manager::ExecutionStatus status = context_->trajectory_execution_manager_->waitForExecution();
    if (status == moveit_controller_manager::ExecutionStatus::SUCCEEDED)
    {
      action_res->error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
    }
    else if (status == moveit_controller_manager::ExecutionStatus::PREEMPTED)
    {
      action_res->error_code.val = moveit_msgs::msg::MoveItErrorCodes::PREEMPTED;
    }
    else if (status == moveit_controller_manager::ExecutionStatus::TIMED_OUT)
    {
      action_res->error_code.val = moveit_msgs::msg::MoveItErrorCodes::TIMED_OUT;
    }
    else
    {
      action_res->error_code.val = moveit_msgs::msg::MoveItErrorCodes::CONTROL_FAILED;
    }
    RCLCPP_INFO_STREAM(LOGGER, "Execution completed: " << status.asString());
  }
  else
  {
    action_res->error_code.val = moveit_msgs::msg::MoveItErrorCodes::CONTROL_FAILED;
  }
}

void MoveGroupAsyncExecuteTrajectoryAction::preemptExecuteTrajectoryCallback()
{
  context_->trajectory_execution_manager_->stopExecution(true);
}

void MoveGroupAsyncExecuteTrajectoryAction::setExecuteTrajectoryState(MoveGroupState state,
                                                                 const std::shared_ptr<ExecTrajectoryGoal>& goal)
{
  auto execute_feedback = std::make_shared<ExecTrajectory::Feedback>();
  execute_feedback->state = stateToStr(state);
  goal->publish_feedback(execute_feedback);
}

}  // namespace move_group

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(move_group::MoveGroupAsyncExecuteTrajectoryAction, move_group::MoveGroupCapability)
