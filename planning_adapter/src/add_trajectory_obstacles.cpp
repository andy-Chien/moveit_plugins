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
#include <moveit/collision_detection_fcl/collision_detector_allocator_fcl.h>

#include "mr_msgs/srv/set_trajectory_state.hpp"
#include "mr_msgs/srv/get_robot_trajectory_obstacle.hpp"

namespace planning_adapter
{
rclcpp::Logger logger(rclcpp::get_logger("plan_adapter.trajectory_obstacles"));
std::string get_obs_service_name("/get_trajectory_obstacle");
std::string set_traj_service_name("/set_trajectory_state");

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
    obs_req->header.stamp = this_node_->now();
    obs_req->robot_name = robot_name_;

    if(!(*planning_scene_))
    {
      *planning_scene_ = std::make_shared<planning_scene::PlanningScene>(
        planning_scene->getRobotModel());
    }
    copyPlanningScene(planning_scene, *planning_scene_);
    (*planning_scene_)->setCurrentState(planning_scene->getCurrentState());
    const collision_detection::WorldPtr& world = (*planning_scene_)->getWorldNonConst();
    // *world = collision_detection::World(*(planning_scene->getWorld()));
    rclcpp::Time t_copy = this_node_->now();
    std::cout<<"============================================================================="<<std::endl;
    std::cout<<"world->size() = "<<world->size()<<std::endl;
    std::cout<<"planning_scene->getWorld()->size() = "<<planning_scene->getWorld()->size()<<std::endl;
    std::cout<<"(*planning_scene_)->getWorld->size() = "<<(*planning_scene_)->getWorld()->size()<<std::endl;

    const auto& clear_and_return = [&world](bool result){
      world->clearObjects();
      return result;
    };

    if(!get_obs_client_->wait_for_service(std::chrono::milliseconds(500))){
      RCLCPP_ERROR(logger,
        "'%s' wait for service '%s' failed!", robot_name_.c_str(), get_obs_service_name.c_str());
      return clear_and_return(false);
    }
    std::shared_future<std::shared_ptr<mr_msgs::srv::GetRobotTrajectoryObstacle_Response>> 
      get_obs_future = get_obs_client_->async_send_request(obs_req).future.share();

    if(get_obs_future.wait_for(std::chrono::milliseconds(500)) != std::future_status::timeout)
    {

      for(const auto& obs : get_obs_future.get()->obstacles_list){
        std::vector<shapes::ShapeConstPtr> shapes;
        EigenSTL::vector_Isometry3d shape_poses;

        if(!shapesAndPosesFromObstacles(obs, shapes, shape_poses)){
          RCLCPP_ERROR(logger, "'%s' add obstacles failed!", robot_name_.c_str());
          return clear_and_return(false);
        }
        const Eigen::Isometry3d& world_to_object_header_transform = 
          planning_scene->getFrameTransform(obs.header.frame_id);
        world->addToObject(
          obs.name, world_to_object_header_transform, shapes, shape_poses);
      }
    }else{
      RCLCPP_ERROR(logger, 
        "'%s' service '%s' time out!", robot_name_.c_str(), get_obs_service_name.c_str());
      return clear_and_return(false);
    }

    rclcpp::Time t_end = this_node_->now();
    RCLCPP_INFO(logger, 
      "Xx=Xx=Xx=Xx=Xx=Xx= '%s' adapt time = %f, copy time = %f, =xX=xX=xX=xX=xX=xX", 
      robot_name_.c_str(), (t_end - t_start).seconds(), (t_copy - t_start).seconds());

    // (*planning_scene_)->allocateCollisionDetector(collision_detection::CollisionDetectorAllocatorFCL::create());
    
    if(!planner(*planning_scene_, req, res)){
      return clear_and_return(false);
    }

    std::cout<<"getCollisionDetectorName() = "<<(*planning_scene_)->getCollisionDetectorName()<<std::endl;
    std::cout<<"getCollisionDetectorName() = "<<planning_scene->getCollisionDetectorName()<<std::endl;
    std::cout<<"isStateColliding = "<<(*planning_scene_)->isStateColliding((*planning_scene_)->getCurrentState(), "ur_manipulator", true)<<std::endl;
    std::cout<<"isStateColliding = "<<planning_scene->isStateColliding(planning_scene->getCurrentState(), "ur_manipulator", true)<<std::endl;
    std::cout<<"world->size() = "<<world->size()<<std::endl;
    std::cout<<"planning_scene->getWorld()->size() = "<<planning_scene->getWorld()->size()<<std::endl;
    std::cout<<"(*planning_scene_)->getWorld->size() = "<<(*planning_scene_)->getWorld()->size()<<std::endl;
    auto obj_ids = (*planning_scene_)->getWorld()->getObjectIds();
    for(const auto& obj_id : obj_ids)
    {
      const auto& obj = *((*planning_scene_)->getWorld()->getObject(obj_id));
      std::cout<<obj.id_<<std::endl;
      Eigen::Matrix3d m = obj.pose_.rotation();
      Eigen::Vector3d v = obj.pose_.translation();
      std::cout << "Rotation: " << std::endl << m << std::endl;
      std::cout << "Translation: " << std::endl << v << std::endl;
      for(auto x : obj.shape_poses_)
      {
        m = x.rotation();
        v = x.translation();
        std::cout << "Rotation: " << std::endl << m << std::endl;
        std::cout << "Translation: " << std::endl << v << std::endl;
      }
      for(auto [a, x] : obj.subframe_poses_)
      {
        m = x.rotation();
        v = x.translation();
        std::cout << "Rotation: " << std::endl << m << std::endl;
        std::cout << "Translation: " << std::endl << v << std::endl;
      }
      std::cout<<"getObjectType(obj_id) = "<<(*planning_scene_)->getObjectType(obj_id).key<<", "<<(*planning_scene_)->getObjectType(obj_id).db<<std::endl;
      // world_out->addToObject(obj.id_, obj.pose_, obj.shapes_, obj.shape_poses_);
      // world_out->setSubframesOfObject(obj.id_, obj.subframe_poses_);
    }
    std::cout<<"---------"<<std::endl;
    obj_ids = planning_scene->getWorld()->getObjectIds();
    for(const auto& obj_id : obj_ids)
    {
      const auto& obj = *(planning_scene->getWorld()->getObject(obj_id));
      std::cout<<obj.id_<<std::endl;
      Eigen::Matrix3d m = obj.pose_.rotation();
      Eigen::Vector3d v = obj.pose_.translation();
      std::cout << "Rotation: " << std::endl << m << std::endl;
      std::cout << "Translation: " << std::endl << v << std::endl;
      for(auto x : obj.shape_poses_)
      {
        m = x.rotation();
        v = x.translation();
        std::cout << "Rotation: " << std::endl << m << std::endl;
        std::cout << "Translation: " << std::endl << v << std::endl;
      }
      for(auto [a, x] : obj.subframe_poses_)
      {
        m = x.rotation();
        v = x.translation();
        std::cout << "Rotation: " << std::endl << m << std::endl;
        std::cout << "Translation: " << std::endl << v << std::endl;
      }
      std::cout<<"getObjectType(obj_id) = "<<planning_scene->getObjectType(obj_id).key<<", "<<planning_scene->getObjectType(obj_id).db<<std::endl;
    }

    moveit_msgs::msg::MotionPlanResponse res_msg;
    res.getMessage(res_msg);
    auto traj_req = std::make_shared<mr_msgs::srv::SetTrajectoryState::Request>();
    traj_req->header.frame_id = robot_name_;
    traj_req->trajectory = res_msg.trajectory.joint_trajectory;
    traj_req->action = mr_msgs::srv::SetTrajectoryState::Request::SET_PLANNED_TRAJECTORY;

    if(!set_traj_client_->wait_for_service(std::chrono::milliseconds(500))){
      RCLCPP_ERROR(logger,
        "'%s' wait for service '%s' failed!", robot_name_.c_str(), set_traj_service_name.c_str());
      res.error_code_.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
      return clear_and_return(false);
    }
    std::shared_future<std::shared_ptr<mr_msgs::srv::SetTrajectoryState_Response>> 
      set_traj_future = set_traj_client_->async_send_request(traj_req).future.share();
    while(set_traj_future.wait_for(std::chrono::milliseconds(500)) == std::future_status::timeout){
      RCLCPP_WARN(logger, 
        "'%s' waiting for the response from set traj service!", robot_name_.c_str());
    }
    if(!set_traj_future.get()->success){
      RCLCPP_ERROR(logger, "'%s' set planned trajectory failed!", robot_name_.c_str());
    }
    return clear_and_return(set_traj_future.get()->success);
  }

  void copyPlanningScene(const planning_scene::PlanningSceneConstPtr& scene_in,
                  planning_scene::PlanningScenePtr& scene_out) const
  {
    const auto& world_in = scene_in->getWorld();
    const auto& world_out = scene_out->getWorldNonConst();
    const auto& state_in = scene_in->getCurrentState();
    const auto& obj_ids = world_in->getObjectIds();

    scene_out->allocateCollisionDetector(collision_detection::CollisionDetectorAllocatorFCL::create());

    scene_out->getCurrentStateNonConst() = state_in;
    scene_out->getTransformsNonConst().setAllTransforms(
      scene_in->getTransforms().getAllTransforms());
    scene_out->getAllowedCollisionMatrixNonConst() = scene_in->getAllowedCollisionMatrix();

    collision_detection::CollisionEnvConstPtr cenv_in = scene_in->getCollisionEnv();
    collision_detection::CollisionEnvPtr cenv_out = scene_out->getCollisionEnvNonConst();
    cenv_out->setLinkScale(cenv_in->getLinkScale());
    cenv_out->setLinkPadding(cenv_in->getLinkPadding());

    scene_out->setCollisionObjectUpdateCallback(
      [](const collision_detection::World::ObjectConstPtr&, 
             collision_detection::World::Action){ return; });
    // scene_out->setAttachedBodyUpdateCallback(moveit::core::AttachedBodyCallback());

    std::vector<const moveit::core::AttachedBody*> attached_objs;
    state_in.getAttachedBodies(attached_objs);

    for (const moveit::core::AttachedBody* attached_obj : attached_objs)
    {
      if(scene_in->hasObjectType(attached_obj->getName())){
        scene_out->setObjectType(attached_obj->getName(), 
          scene_in->getObjectType(attached_obj->getName()));
      }
    }

    for(const auto& obj_id : obj_ids)
    {
      const auto& obj = *(world_in->getObject(obj_id));
      world_out->addToObject(obj.id_, obj.pose_, obj.shapes_, obj.shape_poses_);
      world_out->setSubframesOfObject(obj.id_, obj.subframe_poses_);
      if(scene_in->hasObjectType(obj_id)){
        scene_out->setObjectType(obj_id, scene_in->getObjectType(obj_id));
      }
    }
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
    shapes.reserve(num_shapes);
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
      shapes::ShapeConstPtr s, const mr_msgs::msg::Poses& poses_msg){
      if (!s)
        return;
      for(const auto& pose_msg : poses_msg.poses)
      {
        shape_poses.emplace_back(std::move(pose_msg_to_eigen(pose_msg)));
        shapes.emplace_back(s);
      }
    };

    auto treat_shape_vectors = [&append](const auto& shape_vector,
      const auto& shape_poses_msg, const char* shape_type){
      if(shape_vector.size() != shape_poses_msg.size()){
        RCLCPP_ERROR(logger, "Number of '%s' doesn't match number of it's poses!", shape_type);
        return false;
      }
      for(size_t i=0; i<shape_vector.size(); i++){
        append(shapes::ShapeConstPtr(shapes::constructShapeFromMsg(
          shape_vector[i])), shape_poses_msg[i]);
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
      mr_msgs::srv::SetTrajectoryState>(set_traj_service_name);

    planning_scene_ = new std::shared_ptr<planning_scene::PlanningScene>;

    this_node_thread_ = std::thread([this](){rclcpp::spin(this_node_);});
    
    RCLCPP_INFO(logger, 
      "AddTrajectoryObstacles planning adapter for robot '%s' is loaded", robot_name_.c_str());
  }

  ~AddTrajectoryObstacles() override
  {
    delete planning_scene_;
  }

protected:
  std::string robot_name_;
  std::thread this_node_thread_;
  rclcpp::Node::SharedPtr this_node_;
  std::shared_ptr<planning_scene::PlanningScene>* planning_scene_;
  rclcpp::Client<mr_msgs::srv::SetTrajectoryState>::SharedPtr set_traj_client_;
  rclcpp::Client<mr_msgs::srv::GetRobotTrajectoryObstacle>::SharedPtr get_obs_client_;
};
}  // namespace planning_adapter

CLASS_LOADER_REGISTER_CLASS(planning_adapter::AddTrajectoryObstacles, planning_request_adapter::PlanningRequestAdapter)
