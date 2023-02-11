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
#include <moveit/planning_request_adapter/planning_request_adapter.h>
#include <class_loader/class_loader.hpp>
#include <geometric_shapes/shape_operations.h>
#include <Eigen/Geometry>

#include "mr_msgs/srv/get_robot_trajectory_obstacle.hpp"

namespace planning_adapter
{
rclcpp::Logger logger(rclcpp::get_logger("plan_adapter.trajectory_obstacles"));
std::string service_name("/get_trajectory_obstacle");
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
    rclcpp::Time t1 = this_node_->now();
    auto obs_req = std::make_shared<mr_msgs::srv::GetRobotTrajectoryObstacle::Request>();
    obs_req->header = req.start_state.joint_state.header;
    obs_req->robot_name = robot_name_;

    const auto world_copied = std::make_shared<
      collision_detection::World>(*(planning_scene->getWorld()));
    rclcpp::Time t_cw = this_node_->now();


    if(!client_->wait_for_service(std::chrono::milliseconds(500))){
      RCLCPP_ERROR(logger,
        "Wait for service '%s' failed!", service_name.c_str());
      return false;
    }
    std::shared_future<std::shared_ptr<mr_msgs::srv::GetRobotTrajectoryObstacle_Response>> 
      future = client_->async_send_request(obs_req).future.share();

    if(future.wait_for(std::chrono::milliseconds(500)) != std::future_status::timeout)
    {
      std::vector<shapes::ShapeConstPtr> shapes;
      EigenSTL::vector_Isometry3d shape_poses;

      for(const auto& obs : future.get()->obstacles_list){
        if(!shapesAndPosesFromObstacles(obs, shapes, shape_poses)){
          RCLCPP_ERROR(logger, "Add obstacles failed!");
          return false;
        }
        const Eigen::Isometry3d& world_to_object_header_transform = 
          planning_scene->getFrameTransform(obs.header.frame_id);
        world_copied->addToObject(
          obs.name, world_to_object_header_transform, shapes, shape_poses);
      }
    }else{
      RCLCPP_ERROR(logger, "Service '%s' time out!", service_name.c_str());
      return false;
    }
    rclcpp::Time t_cp = this_node_->now();
    const auto planning_scene_copied = std::make_shared<
      planning_scene::PlanningScene>(planning_scene->getRobotModel(), world_copied);
    planning_scene_copied->setCurrentState(planning_scene->getCurrentState());

    rclcpp::Time t2 = this_node_->now();
    RCLCPP_INFO(logger, "xxxxxx Adapt time = %f, world copy time = %f, scene copy time = %f xxxxxx", 
      (t2 - t1).seconds(), (t_cw - t1).seconds(), (t2 - t_cp).seconds());
    return planner(planning_scene_copied, req, res);
  }

  bool shapesAndPosesFromObstacles(const mr_msgs::msg::Obstacles& obs, 
                    std::vector<shapes::ShapeConstPtr>& shapes,
                    EigenSTL::vector_Isometry3d& shape_poses) const
  {
    std::cout<<"size = "<<obs.meshes.size()<<", "<<obs.meshes_poses.size()<<", "<<
      obs.primitives.size()<<", "<<obs.primitives_poses.size()<<std::endl;

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
        RCLCPP_WARN(logger, "Empty quaternion found in pose message. Setting to neutral orientation.");
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
    client_ = this_node_->create_client<
      mr_msgs::srv::GetRobotTrajectoryObstacle>("/get_trajectory_obstacle");

    this_node_thread_ = std::thread([this](){rclcpp::spin(this_node_);});
    
    RCLCPP_INFO(logger, 
      "AddTrajectoryObstacles planning adapter for robot '%s' is loaded", robot_name_.c_str());
  }

protected:
  std::string robot_name_;
  std::thread this_node_thread_;
  rclcpp::Node::SharedPtr this_node_;
  rclcpp::Client<mr_msgs::srv::GetRobotTrajectoryObstacle>::SharedPtr client_;
};
}  // namespace planning_adapter

CLASS_LOADER_REGISTER_CLASS(planning_adapter::AddTrajectoryObstacles, planning_request_adapter::PlanningRequestAdapter)
