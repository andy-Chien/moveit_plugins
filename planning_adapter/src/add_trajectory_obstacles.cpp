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

#include <thread>
#include <Eigen/Geometry>
#include <class_loader/class_loader.hpp>
#include <geometric_shapes/shape_operations.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/msg/motion_plan_response.hpp>
#include <moveit/planning_request_adapter/planning_request_adapter.h>

#include "mr_msgs/srv/set_planned_trajectory.hpp"
#include "mr_msgs/srv/get_robot_trajectory_obstacle.hpp"

namespace planning_adapter
{
rclcpp::Logger logger(rclcpp::get_logger("plan_adapter.trajectory_obstacles"));
std::string get_obs_service_name("/get_trajectory_obstacle");
std::string set_traj_service_name("/set_planned_trajectory");

class AddTrajectoryObstacles : public planning_request_adapter::PlanningRequestAdapter
{
public:
  std::string getDescription() const override
  {
    return "An adapter to get surrounding robots trajectory obstacle from server";
  }

  bool adaptAndPlan(const PlannerFn& planner, const planning_scene::PlanningSceneConstPtr& planning_scene,
                    const planning_interface::MotionPlanRequest& req, planning_interface::MotionPlanResponse& res,
                    std::vector<std::size_t>& /*added_path_index*/) const override
  {
    rclcpp::Time t_start = this_node_->now();
    auto obs_req = std::make_shared<mr_msgs::srv::GetRobotTrajectoryObstacle::Request>();
    obs_req->header = req.start_state.joint_state.header;
    obs_req->robot_name = robot_name_;

    if(!(*planning_scene_) || !(*world_))
    {
      *world_ = std::make_shared<collision_detection::World>(*(planning_scene->getWorld()));
      *planning_scene_ = std::make_shared<planning_scene::PlanningScene>(
        planning_scene->getRobotModel(), *world_);
    }
    (*planning_scene_)->setCurrentState(planning_scene->getCurrentState());
    **world_ = collision_detection::World(*(planning_scene->getWorld()));
    rclcpp::Time t_copy = this_node_->now();

    if(!get_obs_client_->wait_for_service(std::chrono::milliseconds(500))){
      RCLCPP_ERROR(logger,
        "'%s' wait for service '%s' failed!", robot_name_.c_str(), get_obs_service_name.c_str());
      return false;
    }
    std::shared_future<std::shared_ptr<mr_msgs::srv::GetRobotTrajectoryObstacle_Response>> 
      get_obs_future = get_obs_client_->async_send_request(obs_req).future.share();

    if(get_obs_future.wait_for(std::chrono::milliseconds(500)) != std::future_status::timeout)
    {
      std::vector<shapes::ShapeConstPtr> shapes;
      EigenSTL::vector_Isometry3d shape_poses;

      for(const auto& obs : get_obs_future.get()->obstacles_list){
        if(!shapesAndPosesFromObstacles(obs, shapes, shape_poses)){
          RCLCPP_ERROR(logger, "'%s' add obstacles failed!", robot_name_.c_str());
          return false;
        }
        const Eigen::Isometry3d& world_to_object_header_transform = 
          planning_scene->getFrameTransform(obs.header.frame_id);
        (*world_)->addToObject(
          obs.name, world_to_object_header_transform, shapes, shape_poses);
      }
    }else{
      RCLCPP_ERROR(logger, 
        "'%s' service '%s' time out!", robot_name_.c_str(), get_obs_service_name.c_str());
      return false;
    }

    rclcpp::Time t_end = this_node_->now();
    RCLCPP_INFO(logger, 
      "Xx=Xx=Xx=Xx=Xx=Xx= '%s' adapt time = %f, copy time = %f, =xX=xX=xX=xX=xX=xX", 
      robot_name_.c_str(), (t_end - t_start).seconds(), (t_copy - t_start).seconds());

    if(!planner(*planning_scene_, req, res)){
      return false;
    }

    moveit_msgs::msg::MotionPlanResponse res_msg;
    res.getMessage(res_msg);
    auto traj_req = std::make_shared<mr_msgs::srv::SetPlannedTrajectory::Request>();
    traj_req->trajectory = res_msg.trajectory.joint_trajectory;
    traj_req->robot_name = robot_name_;

    if(!set_traj_client_->wait_for_service(std::chrono::milliseconds(500))){
      RCLCPP_ERROR(logger,
        "'%s' wait for service '%s' failed!", robot_name_.c_str(), set_traj_service_name.c_str());
      res.error_code_.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
      return false;
    }
    std::shared_future<std::shared_ptr<mr_msgs::srv::SetPlannedTrajectory_Response>> 
      set_traj_future = set_traj_client_->async_send_request(traj_req).future.share();
    while(set_traj_future.wait_for(std::chrono::milliseconds(500)) == std::future_status::timeout){
      RCLCPP_WARN(logger, 
        "'%s' waiting for the response from set traj service!", robot_name_.c_str());
    }
    if(!set_traj_future.get()->success){
      RCLCPP_ERROR(logger, "'%s' set planned trajectory failed!", robot_name_.c_str());
    }
    return set_traj_future.get()->success;
  }

  bool shapesAndPosesFromObstacles(const mr_msgs::msg::Obstacles& obs, 
                    std::vector<shapes::ShapeConstPtr>& shapes,
                    EigenSTL::vector_Isometry3d& shape_poses) const
  {
    std::cout<<"size = "<<obs.meshes.size()<<", "<<obs.meshes_poses.size()<<", "<<
      obs.primitives.size()<<", "<<obs.primitives_poses.size()<<std::endl;
    if(obs.meshes_poses.size() > 0){
      std::cout<<"obs.meshes_poses[0].poses.size() = "<<obs.meshes_poses[0].poses.size()<<std::endl;
    }

    const size_t num_shapes = obs.primitives.size() + obs.meshes.size();
    shapes.clear();
    shapes.reserve(num_shapes);
    shape_poses.clear();
    shape_poses.reserve(num_shapes);

    const auto& pose_msg_to_eigen = [](const mr_msgs::msg::Pose& msg)
    {
      Eigen::Translation3d trans(msg.pose[0], msg.pose[1], msg.pose[2]);
      Eigen::Quaterniond quat(msg.pose[3], msg.pose[4], msg.pose[5], msg.pose[6]);
      if ((quat.x() == 0) && (quat.y() == 0) && (quat.z() == 0) && (quat.w() == 0)){
        RCLCPP_WARN(logger,
          "Empty quaternion found in pose message. Setting to neutral orientation.");
        quat.setIdentity();
      }else{
        quat.normalize();
      }
      return Eigen::Isometry3d(trans * quat);
    };

    auto append = [&shapes, &shape_poses, &pose_msg_to_eigen](
      shapes::Shape* s, const mr_msgs::msg::Poses& poses_msg){
      if (!s)
        return;
      for(const auto& pose_msg : poses_msg.poses)
      {
        shape_poses.emplace_back(std::move(pose_msg_to_eigen(pose_msg)));
        shapes.emplace_back(shapes::ShapeConstPtr(s));
      }
    };

    auto treat_shape_vectors = [&append](const auto& shape_vector,
      const auto& shape_poses_msg, const char* shape_type){
      if(shape_vector.size() != shape_poses_msg.size()){
        RCLCPP_ERROR(logger, "Number of '%s' doesn't match number of it's poses!", shape_type);
        return false;
      }
      for(size_t i=0; i<shape_vector.size(); i++){
        append(shapes::constructShapeFromMsg(shape_vector[i]), shape_poses_msg[i]);
      }
      return true;
    };

    return (treat_shape_vectors(obs.meshes, obs.meshes_poses, "MESH") &&
            treat_shape_vectors(obs.primitives, obs.primitives_poses, "PRIMITIVES"));
  }

  void initialize(const rclcpp::Node::SharedPtr& node, 
                  const std::string& /* parameter_namespace */) override
  {
    this_node_ = std::make_shared<rclcpp::Node>("tarjectory_obstacles_getter");
    
    const auto& name_without_slash = [&](const char* ns){
      const auto& name = std::string(ns);
      if(name.size() > 1 && name.at(0) == '/'){
        return std::string(name.begin() + 1, name.end());
      }else{
        return name;
      }
    };

    robot_name_ = name_without_slash(node->get_namespace());
    get_obs_client_ = this_node_->create_client<
      mr_msgs::srv::GetRobotTrajectoryObstacle>(get_obs_service_name);

    set_traj_client_ = this_node_->create_client<
      mr_msgs::srv::SetPlannedTrajectory>(set_traj_service_name);

    world_ = new std::shared_ptr<collision_detection::World>;
    planning_scene_ = new std::shared_ptr<planning_scene::PlanningScene>;

    this_node_thread_ = std::thread([this](){rclcpp::spin(this_node_);});
    
    RCLCPP_INFO(logger, 
      "AddTrajectoryObstacles planning adapter for robot '%s' is loaded", robot_name_.c_str());
  }

  ~AddTrajectoryObstacles() override
  {
    delete world_;
    delete planning_scene_;
  }

protected:
  std::string robot_name_;
  std::thread this_node_thread_;
  rclcpp::Node::SharedPtr this_node_;
  std::shared_ptr<collision_detection::World>* world_;
  std::shared_ptr<planning_scene::PlanningScene>* planning_scene_;
  rclcpp::Client<mr_msgs::srv::SetPlannedTrajectory>::SharedPtr set_traj_client_;
  rclcpp::Client<mr_msgs::srv::GetRobotTrajectoryObstacle>::SharedPtr get_obs_client_;
};
}  // namespace planning_adapter

CLASS_LOADER_REGISTER_CLASS(planning_adapter::AddTrajectoryObstacles, planning_request_adapter::PlanningRequestAdapter)
