#include <Eigen/Geometry>
#include <geometric_shapes/shape_operations.h>

#include "scene_buffer/scene_buffer.hpp"

SceneBuffer::SceneBuffer(const std::string& node_name, const rclcpp::NodeOptions& node_options)
: Node(node_name, node_options)
{
  cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  get_obstacle_service_ = this->create_service<ObstacleSrv>(
    "/get_trajectory_obstacle", std::bind(
      &SceneBuffer::get_obstacle_cb, this, std::placeholders::_1, std::placeholders::_2), 
      rmw_qos_profile_services_default, cb_group_);
  set_trajectory_service_ = this->create_service<TrajectorySrv>(
    "/set_trajectory_state", std::bind(
      &SceneBuffer::set_trajectory_cb, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, cb_group_);
  obstacles_visualization_publisher_ = this->create_publisher<MarkerArray>(
    "/visualization_marker_array", 1);
}

void SceneBuffer::init()
{
  // Create the parameter listener and get the parameters
  param_listener_ = std::make_shared<ParamListener>(shared_from_this());
  params_ = param_listener_->get_params();
  delay_duration_ = std::make_unique<rclcpp::Duration>(
    std::chrono::nanoseconds(int32_t(params_.delay_duration * 1e9)));
  for(const auto& robot_name : params_.robot_names)
  {
    auto robot = std::make_shared<Robot>(shared_from_this(), robot_name, params_.padding);
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

  // std::vector<Eigen::Isometry3d> link_poses;
  // std::vector<std::vector<Eigen::Isometry3d>> link_poses_;

  robot_model_loader::RobotModelLoader rml(
    node_, robot_model_loader::RobotModelLoader::Options(urdf, srdf));
  model = rml.getModel();
  state = std::make_shared<moveit::core::RobotState>(model);

  state->setToDefaultValues();
  std::cout<<"ccccccccccccccccccccccccccccccccccccccccccccccccccc"<<std::endl;
  model->printModelInfo(std::cout);
  std::cout<<"ddddddddddddddddddddddddddddddddddddddddddddddddddd"<<std::endl;
  state->printTransforms();

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
  obstacles.header.frame_id = "world";
  obstacles.meshes_poses.resize(mesh_links.size());
  obstacles.primitives_poses.resize(prim_links.size());

  if(!obstacles_from_links()){
    RCLCPP_ERROR(node_->get_logger(), 
      "Convert from link to obstcale msg failed");
  }
}

void SceneBuffer::pub_obstacles(const Robot& robot, const uint8_t step) const
{
  const auto& obs = robot.obstacles;
  const auto& links = robot.mesh_links;
  const auto time_now = this->now();

  MarkerArray msg;

  size_t num_obs = 0;
  for(const auto& poses : obs.meshes_poses){
    num_obs += poses.poses.size();
  }
  msg.markers.reserve(num_obs);

  for(size_t i=0; i<links.size(); i++)
  {
    const std::string& mesh_file_name = links[i]->getVisualMeshFilename();
    const std::string& link_name = links[i]->getName();
    for(size_t id=0; id < obs.meshes_poses[i].poses.size(); id++)
    {
      if(id % step != 0 && id != obs.meshes_poses[i].poses.size() - 1){
        continue;
      }
      const auto& pose = obs.meshes_poses[i].poses[id];
      visualization_msgs::msg::Marker marker;
      marker.header.stamp = time_now;
      marker.header.frame_id = "world";
      marker.ns = link_name;
      marker.id = id;
      marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.lifetime = rclcpp::Duration(std::chrono::seconds(3));
      marker.frame_locked = false;
      marker.mesh_resource = mesh_file_name;
      marker.mesh_use_embedded_materials = true;
      marker.pose.position.x = pose.pose[0];
      marker.pose.position.y = pose.pose[1];
      marker.pose.position.z = pose.pose[2];
      marker.pose.orientation.w = pose.pose[3];
      marker.pose.orientation.x = pose.pose[4];
      marker.pose.orientation.y = pose.pose[5];
      marker.pose.orientation.z = pose.pose[6];
      marker.scale.x = 1;
      marker.scale.y = 1;
      marker.scale.z = 1;
      marker.color.r = 0.2;
      marker.color.g = 0.8;
      marker.color.b = 0.5;
      marker.color.a = 0.1;
      msg.markers.emplace_back(std::move(marker));
    }
  }
  obstacles_visualization_publisher_->publish(msg);
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

  const auto& pose_interpolation = [](const int interpolation_num, 
    const Eigen::Quaterniond q_0, const Eigen::Vector3d v_0, const Eigen::Quaterniond q_1, 
    const Eigen::Vector3d v_1, std::vector<Eigen::Isometry3d>& trans){
      std::cout<<"pose_interpolation: "<<interpolation_num<<std::endl;
      for(int i=1; i < interpolation_num; i++)
      {
        const double t = double(i) / double(interpolation_num);
        const Eigen::Vector3d v_interp = (1 - t) * v_1 + t * v_0;
        const Eigen::Quaterniond q_interp = q_1.slerp(t, q_0);
        trans.emplace_back(Eigen::Translation3d(v_interp(0), v_interp(1), v_interp(2)) * q_interp);
      }
    };

  const double p_min = params_.pos_diff_min;
  const double p_max = params_.pos_diff_max;
  const double q_min = params_.rot_diff_min;
  const double q_max = params_.rot_diff_max;
  const auto& pose_adjust = [&p_min, &p_max, &q_min, &q_max, &pose_interpolation]
    (std::vector<Eigen::Isometry3d>& trans, const Eigen::Isometry3d& last_trans){
      assert(trans.size() == 1);
      const Eigen::Quaterniond q_0(last_trans.rotation());
      const Eigen::Vector3d v_0(last_trans.translation());
      const Eigen::Quaterniond q_1(trans[0].rotation());
      const Eigen::Vector3d v_1(trans[0].translation());
      const auto p_d = v_1 - v_0;
      const double q_d[4] = {q_1.w()-q_0.w(), q_1.x()-q_0.x(), q_1.y()-q_0.y(), q_1.z()-q_0.z()};
      const double p_l = std::hypot(p_d(0), p_d(1), p_d(2));
      const double q_l_0 = q_d[0]*q_d[0] + q_d[1]*q_d[1] + q_d[2]*q_d[2] + q_d[3]*q_d[3];
      const double q_l = (q_l_0 > 2) ? 4 - q_l_0 : q_l_0; 
      if(p_l < p_min && q_l < q_min){
        return false; // ignore this trans
      }
      if(p_l > p_max || q_l > q_max){
        const int inter_num = 
          std::ceil(std::max(p_l / ((p_max + p_min) / 2), q_l / ((q_max + q_min) / 2)));
        pose_interpolation(inter_num, q_0, v_0, q_1, v_1, trans);
      }
      return true;
    };

  const auto& get_tarj_start_time = [](const auto& trajectory){
    return rclcpp::Time(trajectory->header.stamp);
  };
  const auto& tarj_last_time = [&get_tarj_start_time](const auto& trajectory){
    return rclcpp::Time(get_tarj_start_time(trajectory) + rclcpp::Duration(
      trajectory->points.back().time_from_start));
  };

  const auto& get_link_poses_from_state = [&eigen_to_msg, &pose_adjust](
    const std::shared_ptr<Robot>& robot, std::vector<Eigen::Isometry3d>& last_trans){
      const size_t mesh_links_size = robot->mesh_links.size();
      const size_t prim_links_size = robot->prim_links.size();
      const bool check_diff = last_trans.size() != 0;
      if(last_trans.size() == 0){
        last_trans.resize(mesh_links_size + prim_links_size);
      }
      assert(last_trans.size() == mesh_links_size + prim_links_size);
      const moveit::core::RobotStateConstPtr const_state(robot->state);
      std::vector<Eigen::Isometry3d> trans_list;
      trans_list.reserve(10);
      for(size_t i=0; i < mesh_links_size; i++){
        trans_list.clear();
        trans_list.emplace_back(const_state->getCollisionBodyTransform(robot->mesh_links[i], 0));
        if(check_diff && !pose_adjust(trans_list, last_trans[i])){
          continue;
        }
        for(const auto& trans : trans_list){
          robot->obstacles.meshes_poses[i].poses.push_back(eigen_to_msg(trans));
        }
        last_trans[i] = trans_list[0];
      }
      for(size_t i=0; i < prim_links_size; i++){
        trans_list.clear();
        trans_list.emplace_back(const_state->getCollisionBodyTransform(robot->prim_links[i], 0));
        if(check_diff && !pose_adjust(trans_list, last_trans[i + mesh_links_size])){
          continue;
        }
        for(const auto& trans : trans_list){
          robot->obstacles.primitives_poses[i].poses.push_back(eigen_to_msg(trans));
        }
        last_trans[i + mesh_links_size] = trans_list[0];
      }
    };

  const auto& check_last_time = 
    [&get_tarj_start_time, &tarj_last_time](
      const rclcpp::Time& t, const rclcpp::Duration& delay_time, const auto& traj){
      return (get_tarj_start_time(traj).nanoseconds() > 
        0 && t > tarj_last_time(traj) + delay_time);
    };

  const auto& check_running_time_with_delay = 
    [this](const rclcpp::Time& start_time, const rclcpp::Time& tarj_start_time, const auto& point){
      return start_time > (tarj_start_time + 
        rclcpp::Duration(point.time_from_start) + *delay_duration_);
    };

  const auto& get_pose_from_trajectory = 
    [&check_running_time_with_delay, &get_link_poses_from_state, &get_tarj_start_time](
      const auto& robot, const auto&  trajectory, const auto& start_time, bool running_traj){
        if(!trajectory){
          return;
        }
        std::vector<Eigen::Isometry3d> last_trans;
        const auto& traj_joint_names = trajectory->joint_names;
        const auto& tarj_start_time = get_tarj_start_time(trajectory);
        for(const auto& point : trajectory->points)
        {
          if(running_traj && 
            check_running_time_with_delay(start_time, tarj_start_time, point)){
            continue;
          }
          for(size_t i=0; i<traj_joint_names.size(); i++)
          {
            robot->state->setJointPositions(
              traj_joint_names[i], &(point.positions[i]));
          }
          robot->state->update();
          get_link_poses_from_state(robot, last_trans);
        }
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

  const auto try_to_set_some_one_is_planning = [this](const std::string& name){
    const std::lock_guard<std::mutex> planning_lock(some_one_is_planning_mutex_);
    if(some_one_is_planning_ == ""){
      some_one_is_planning_ = name;
      return true;
    }else{
      return false;
    }
  };

  while(!try_to_set_some_one_is_planning(req_robot_name)){
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    RCLCPP_INFO(get_logger(), "Waiting for other planning");
  }

  const rclcpp::Time start_time = this->now();

  const auto& collision_robots =
    params_.collision_maps.robot_names_map.at(req_robot_name).collision_robots;

  res->obstacles_list.reserve(collision_robots.size());

  for(const auto& other_name : collision_robots)
  {

    const auto& other_robot = robots_.at(other_name);
    other_robot->clean_poses();

    const std::shared_lock<std::shared_mutex> data_lock(other_robot->get_trajectory_data_mutex());

    if(!(other_robot->planned_trajectory || other_robot->running_trajectory))
    {
      other_robot->update_to_current();
      std::vector<Eigen::Isometry3d> last_trans;
      get_link_poses_from_state(other_robot, last_trans);
      res->obstacles_list.push_back(other_robot->obstacles);
      continue;
    }

    get_pose_from_trajectory(other_robot, other_robot->running_trajectory, start_time, true);
    get_pose_from_trajectory(other_robot, other_robot->planned_trajectory, start_time, false);

    std::cout<<"mesh_poses.poses.size() = "<<std::endl;
    for(const auto& mesh_poses : other_robot->obstacles.meshes_poses)
      std::cout<<mesh_poses.poses.size()<<", ";
    std::cout<<std::endl;

    res->obstacles_list.push_back(other_robot->obstacles);
  }
  if(params_.pub_obstacles){
    for(const auto& other_name : collision_robots){
      pub_obstacles(*robots_.at(other_name), params_.obstacle_pub_step);
    }
  }
  rclcpp::Time t2 = this->now();
  std::cout<<"get_obstacle_cb end time = "<<(t2 - t1).seconds()<<std::endl;
  return true;
}

bool SceneBuffer::Robot::obstacles_from_links()
{
  obstacles.meshes.reserve(mesh_links.size());
  obstacles.primitives.reserve(prim_links.size());

  const auto&& mesh_msg_from_shape = [&](const shapes::Mesh* mesh_in){
    auto mesh = *mesh_in;
    mesh.padd(padding_);
    shape_msgs::msg::Mesh mesh_msg;
    mesh_msg.triangles.resize(mesh.triangle_count);
    mesh_msg.vertices.resize(mesh.vertex_count);

    for(size_t i=0; i < mesh.triangle_count; i++){
      mesh_msg.triangles[i].vertex_indices[0] = mesh.triangles[3 * i];
      mesh_msg.triangles[i].vertex_indices[1] = mesh.triangles[3 * i + 1];
      mesh_msg.triangles[i].vertex_indices[2] = mesh.triangles[3 * i + 2];
    }
    for(size_t i=0; i < mesh.vertex_count; i++)
    {
      mesh_msg.vertices[i].x = mesh.vertices[3 * i];
      mesh_msg.vertices[i].y = mesh.vertices[3 * i + 1];
      mesh_msg.vertices[i].z = mesh.vertices[3 * i + 2];
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
      auto box = *static_cast<const shapes::Box*>(shape.get());
      box.padd(padding_);
      msg.type = shape_msgs::msg::SolidPrimitive::BOX;
      msg.dimensions.resize(3);
      msg.dimensions[0] = box.size[0];
      msg.dimensions[1] = box.size[1];
      msg.dimensions[2] = box.size[2];
      break;
    }
    case shapes::SPHERE:
    {
      auto sphere = *static_cast<const shapes::Sphere*>(shape.get());
      sphere.padd(padding_);
      msg.type = shape_msgs::msg::SolidPrimitive::SPHERE;
      msg.dimensions.resize(1);
      msg.dimensions[0] = sphere.radius;
      break;
    }
    case shapes::CYLINDER:
    {
      auto cylinder = *static_cast<const shapes::Cylinder*>(shape.get());
      cylinder.padd(padding_);
      msg.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
      msg.dimensions.resize(2);
      msg.dimensions[1] = cylinder.length;
      msg.dimensions[0] = cylinder.radius;
      break;
    }
    case shapes::CONE:
    {
      auto cone = *static_cast<const shapes::Cone*>(shape.get());
      cone.padd(padding_);
      msg.type = shape_msgs::msg::SolidPrimitive::CONE;
      msg.dimensions.resize(2);
      msg.dimensions[1] = cone.length;
      msg.dimensions[0] = cone.radius;
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
  const auto try_to_release_some_one_is_planning = [this](const std::string& name){
    const std::lock_guard<std::mutex> planning_lock(some_one_is_planning_mutex_);
    if(some_one_is_planning_ == name){
      some_one_is_planning_ = "";
    }else{
      throw std::runtime_error("mutex name not the same when try to release, mutex name: '" + 
        some_one_is_planning_ + "', try to release '" + name + "'.");
    }
  };

  const std::string req_robot_name = [](std::string& robot_name){
    if(robot_name.size() > 1 && robot_name.at(0) == '/'){
      return std::string(robot_name.begin() + 1, robot_name.end());
    }else{
      return robot_name;
    }
  }(req->header.frame_id);

  const auto& robot = robots_.at(req_robot_name);
  const std::lock_guard<std::shared_mutex> data_lock(robot->get_trajectory_data_mutex());
  
  res->success = true;
  switch (req->action)
  {
  case TrajectorySrv::Request::SET_PLANNED_TRAJECTORY:
    robot->planned_trajectory = 
      std::make_shared<TrajectoryMsg>(std::move(req->trajectory));
    try_to_release_some_one_is_planning(req_robot_name);
    break;

  case TrajectorySrv::Request::PLANNING_FAILED:
    try_to_release_some_one_is_planning(req_robot_name);
    break;

  case TrajectorySrv::Request::MARK_TRAJECTORY_START_TIME:
    robot->running_trajectory = robot->planned_trajectory;
    robot->running_trajectory->header.stamp = req->header.stamp;
    robot->planned_trajectory = nullptr;
    break;

  case TrajectorySrv::Request::TRAJECTORY_DONE_OR_CANCEL:
    robot->running_trajectory = nullptr;
    break;
  
  case TrajectorySrv::Request::ERASE_PLANNED_TRAJECTORY:
    robot->planned_trajectory = nullptr;
    break;
  
  default:
    res->success = false;
    RCLCPP_ERROR(get_logger(), "Unknown trajectory setting action!");
    break;
  }
  return true;
}