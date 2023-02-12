#include <Eigen/Geometry>
#include <geometric_shapes/shape_operations.h>

#include "scene_buffer/scene_buffer.hpp"

SceneBuffer::SceneBuffer(const std::string& node_name, const rclcpp::NodeOptions& node_options)
: Node(node_name, node_options)
{
  get_obstacle_service_ = this->create_service<ObstacleSrv>(
    "get_trajectory_obstacle", std::bind(
      &SceneBuffer::get_obstacle_cb, this, std::placeholders::_1, std::placeholders::_2));
  set_trajectory_service_ = this->create_service<TrajectorySrv>(
    "set_planned_trajectory", std::bind(
      &SceneBuffer::set_trajectory_cb, this, std::placeholders::_1, std::placeholders::_2));
}

void SceneBuffer::init()
{
  // Create the parameter listener and get the parameters
  param_listener_ = std::make_shared<ParamListener>(shared_from_this());
  params_ = param_listener_->get_params();
  rclcpp::SubscriptionOptions options;
  for(const auto& robot_name : params_.robot_names)
  {
    auto robot = std::make_shared<Robot>(shared_from_this(), robot_name);
    robots_.insert(std::pair<std::string, std::shared_ptr<Robot>>(
      robot_name, robot)
    );
  }
}

void SceneBuffer::Robot::load_robot(const std::string& robot_name)
{
  param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
    node_, robot_name + "/move_group");
  while(!param_client_->wait_for_service(std::chrono::milliseconds(500))){
    RCLCPP_INFO(node_->get_logger(), "Waiting for get param service.");
  }

  const std::vector<std::string> params_name(
    {"robot_description", "robot_description_semantic"});

  param_client_->get_parameters(params_name,
    [this](std::shared_future<std::vector<rclcpp::Parameter>> future) -> void {
      const auto& params = future.get();
      this->load_robot(params.at(0).as_string(), params.at(1).as_string());
    }
  );
}

void SceneBuffer::Robot::load_robot(const std::string& urdf, const std::string& srdf)
{
  std::cout<<urdf<<std::endl;
  std::cout<<"XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX"<<std::endl;
  std::cout<<srdf<<std::endl;

  std::vector<Eigen::Isometry3d> link_poses;
  std::vector<std::vector<Eigen::Isometry3d>> link_poses_;

  robot_model_loader::RobotModelLoader rml(
    node_, robot_model_loader::RobotModelLoader::Options(urdf, srdf));
  model = rml.getModel();
  state = std::make_shared<moveit::core::RobotState>(model);

  state->setToDefaultValues();
  std::cout<<"ccccccccccccccccccccccccccccccccccccccccccccccccccc"<<std::endl;
  model->printModelInfo(std::cout);
  std::cout<<"ddddddddddddddddddddddddddddddddddddddddddddddddddd"<<std::endl;
  state->printTransforms();
  double ang = -0.1;
  state->setJointPositions("shoulder_lift_joint", &ang);
  state->update();
  {
    std::cout<<"BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB"<<std::endl;
    const auto& links = model->getLinkModels();
    for(const auto& link : links)
    {
      const auto& shap = link->getShapes();
      std::cout << "shap size = " << shap.size() << std::endl;
      const auto& trans = state->getGlobalLinkTransform(link);
      const Eigen::Matrix3d& m = trans.rotation();
      const Eigen::Vector3d& v = trans.translation();
      std::cout << "Rotation: " << std::endl << m << std::endl;
      std::cout << "Translation: " << std::endl << v << std::endl;
      link_poses.push_back(trans);
    }
    link_poses_.push_back(link_poses);
    link_poses.clear();
    std::cout<<"BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB"<<std::endl;
  }
  std::cout<<"eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee"<<std::endl;
  state->printTransforms();
  ang = 0.1;
  state->setJointPositions("shoulder_lift_joint", &ang);
  state->update();
  std::cout<<"ffffffffffffffffffffffffffffffffffffffffffffffffffff"<<std::endl;
  state->printTransforms();

  {
    std::cout<<"DDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDD"<<std::endl;
    const auto links = model->getLinkModels();
    for(const auto link : links)
    {
      const auto& shap = link->getShapes();
      std::cout << "shap size = " << shap.size() << std::endl;
      const auto& trans = state->getGlobalLinkTransform(link);
      const Eigen::Matrix3d& m = trans.rotation();
      const Eigen::Vector3d& v = trans.translation();
      std::cout << "Rotation: " << std::endl << m << std::endl;
      std::cout << "Translation: " << std::endl << v << std::endl;
      link_poses.push_back(trans);
    }
    link_poses_.push_back(link_poses);
    link_poses.clear();
    std::cout<<"DDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDD"<<std::endl;
  }
  std::cout<<"size of link_poses = "<<link_poses_.size()<<" x "<<link_poses_[0].size()<<std::endl;

  for(const auto& link : model->getLinkModels())
  {
    if(link->getShapes().size() == 0){
      continue;
    }
    if(link->getShapes().size() > 1){
      RCLCPP_ERROR(node_->get_logger(), 
        "Now it only support one shape for each link");
      return;
    }
    const auto& type = link->getShapes().at(0)->type;

    if(type == shapes::MESH)      {
      mesh_links.push_back(link);
    }
    else if(type == shapes::BOX || type == shapes::CONE || 
            type == shapes::CYLINDER || type == shapes::SPHERE){
      prim_links.push_back(link);
    }
  }
  obstacles.meshes_poses.resize(mesh_links.size());
  obstacles.primitives_poses.resize(prim_links.size());

  if(!obstacles_from_links()){
    RCLCPP_ERROR(node_->get_logger(), 
      "Convert from link to obstcale msg failed");
  }
}

bool SceneBuffer::get_obstacle_cb(
  const std::shared_ptr<ObstacleSrv::Request> req,
  std::shared_ptr<ObstacleSrv::Response> res)
{
  rclcpp::Time t1 = this->now();

  const auto& eigen_to_msg = [](const Eigen::Isometry3d& trans){
    const Eigen::Quaterniond q(trans.rotation());
    const Eigen::Vector3d v(trans.translation());
    mr_msgs::msg::Pose p;
    p.pose[0] = v(0);
    p.pose[1] = v(1);
    p.pose[2] = v(2);
    p.pose[3] = q.w();
    p.pose[4] = q.x();
    p.pose[5] = q.y();
    p.pose[6] = q.z();
    return p;
  };

  const auto& tarj_start_time = [](const auto& trajectory){
    return rclcpp::Time(trajectory->header.stamp);
  };
  const auto& tarj_last_time = [&tarj_start_time](const auto& trajectory){
    return rclcpp::Time(tarj_start_time(trajectory) + rclcpp::Duration(
      trajectory->points.back().time_from_start));
  };

  const auto& get_link_poses_from_state = 
    [&eigen_to_msg](const std::shared_ptr<Robot>& robot){
      for(size_t i=0; i<robot->mesh_links.size(); i++){
        const auto& trans = robot->state->getGlobalLinkTransform(robot->mesh_links[i]);
        robot->obstacles.meshes_poses[i].poses.push_back(eigen_to_msg(trans));
      }
      for(size_t i=0; i<robot->prim_links.size(); i++){
        const auto& trans = robot->state->getGlobalLinkTransform(robot->prim_links[i]);
        robot->obstacles.primitives_poses[i].poses.push_back(eigen_to_msg(trans));
      }
    };

  const auto& check_last_time = 
    [&tarj_start_time, &tarj_last_time](const rclcpp::Time t, const auto& traj){
      return tarj_start_time(traj).nanoseconds() > 0 && t > tarj_last_time(traj);
    };

  const auto& check_all_last_time = 
    [&check_last_time](const rclcpp::Time t, const auto& trajectories){
      for(const auto& traj : trajectories){
        if(!check_last_time(t, traj)){
          return false;
        }
      }
      return true;
    };

  const std::string req_robot_name = [](std::string& robot_name){
    if(robot_name.size() > 1 && robot_name.at(0) == '/'){
      return std::string(robot_name.begin() + 1, robot_name.end());
    }else{
      return robot_name;
    }
  }(req->robot_name);

  if(robots_.find(req_robot_name) == robots_.end()){
    RCLCPP_ERROR(get_logger(), "Requested robot has not been registered");
    return false;
  }

  const rclcpp::Time start_time(
    rclcpp::Time(req->header.stamp) + rclcpp::Duration(req->run_after));

  const auto& collision_robots =
    params_.collision_maps.robot_names_map.at(req_robot_name).collision_robots;

  res->obstacles_list.reserve(collision_robots.size());

  for(const auto& other_name : collision_robots)
  {
    const auto& other_robot = robots_.at(other_name);
    other_robot->clean_poses();

    std::cout<<"other_name = "<<other_name<<", traj size = "<<
      other_robot->trajectories.size()<<std::endl;

    if(other_robot->trajectories.size() == 0 || 
      check_all_last_time(start_time, other_robot->trajectories))
    {
      other_robot->update_to_current();
      get_link_poses_from_state(other_robot);
      res->obstacles_list.push_back(other_robot->obstacles);
      continue;
    }

    for(const auto& trajectory : other_robot->trajectories)
    {
      const auto& traj_joint_names = trajectory->joint_names;

      for(const auto& point : trajectory->points)
      {
        if(start_time > (tarj_start_time(trajectory) + rclcpp::Duration(point.time_from_start))){
          continue;
        }
        for(size_t i=0; i<traj_joint_names.size(); i++)
        {
          other_robot->state->setJointPositions(
            traj_joint_names[i], &(point.positions[i]));
        }
        other_robot->state->update();
        get_link_poses_from_state(other_robot);
      }
    }
    res->obstacles_list.push_back(other_robot->obstacles);
  }
  rclcpp::Time t2 = this->now();
  std::cout<<"get_obstacle_cb end time = "<<(t2 - t1).seconds()<<std::endl;
  return true;
}

bool SceneBuffer::Robot::obstacles_from_links()
{
  obstacles.meshes.reserve(mesh_links.size());
  obstacles.primitives.reserve(prim_links.size());

  const auto&& mesh_msg_from_shape = [&](const shapes::Mesh* mesh){
    shape_msgs::msg::Mesh mesh_msg;
    mesh_msg.triangles.resize(mesh->triangle_count);
    mesh_msg.vertices.resize(mesh->vertex_count);

    for(size_t i=0; i < mesh->triangle_count; i++){
      mesh_msg.triangles[i].vertex_indices[0] = mesh->triangles[3 * i];
      mesh_msg.triangles[i].vertex_indices[1] = mesh->triangles[3 * i + 1];
      mesh_msg.triangles[i].vertex_indices[2] = mesh->triangles[3 * i + 2];
    }
    for(size_t i=0; i < mesh->vertex_count; i++)
    {
      mesh_msg.vertices[i].x = mesh->vertices[3 * i];
      mesh_msg.vertices[i].y = mesh->vertices[3 * i + 1];
      mesh_msg.vertices[i].z = mesh->vertices[3 * i + 2];
    }
    return mesh_msg;
  };

  const auto&& solid_msg_from_shape = [&](const shapes::ShapeConstPtr shape){
    shape_msgs::msg::SolidPrimitive msg;
    const auto type = shape->type;
    switch(type)
    {
    case shapes::BOX:
    {
      const auto box = static_cast<const shapes::Box*>(shape.get());
      msg.type = shape_msgs::msg::SolidPrimitive::BOX;
      msg.dimensions.resize(3);
      msg.dimensions[0] = box->size[0];
      msg.dimensions[1] = box->size[1];
      msg.dimensions[2] = box->size[2];
      break;
    }
    case shapes::SPHERE:
    {
      const auto sphere = static_cast<const shapes::Sphere*>(shape.get());
      msg.type = shape_msgs::msg::SolidPrimitive::SPHERE;
      msg.dimensions.resize(1);
      msg.dimensions[0] = sphere->radius;
      break;
    }
    case shapes::CYLINDER:
    {
      const auto cylinder = static_cast<const shapes::Cylinder*>(shape.get());
      msg.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
      msg.dimensions.resize(2);
      msg.dimensions[1] = cylinder->length;
      msg.dimensions[0] = cylinder->radius;
      break;
    }
    case shapes::CONE:
    {
      const auto cone = static_cast<const shapes::Cone*>(shape.get());
      msg.type = shape_msgs::msg::SolidPrimitive::CONE;
      msg.dimensions.resize(2);
      msg.dimensions[1] = cone->length;
      msg.dimensions[0] = cone->radius;
      break;
    }
    default:
      break;
    }
    return msg;
  };

  for(const auto& link : mesh_links){
    obstacles.meshes.push_back(std::move(mesh_msg_from_shape(
      static_cast<const shapes::Mesh*>(link->getShapes().at(0).get()))
    ));
  }
  for(const auto& link : prim_links){
    obstacles.primitives.push_back(std::move(
      solid_msg_from_shape(link->getShapes().at(0))));
  }
  return true;
}

bool SceneBuffer::set_trajectory_cb(
  const std::shared_ptr<TrajectorySrv::Request> req,
  std::shared_ptr<TrajectorySrv::Response> res)
{
  robots_.at(req->robot_name)->trajectories.push_back(
    std::make_shared<TrajectoryMsg>(std::move(req->trajectory)));
  
  res->success = true;
  return true;
}