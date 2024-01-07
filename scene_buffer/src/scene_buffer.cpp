#include <Eigen/Geometry>
#include <geometric_shapes/shape_operations.h>

#include "scene_buffer/scene_buffer.hpp"

static const rclcpp::Logger ROBOT_LOGGER = rclcpp::get_logger("scene_buffer_robot");
static const rclcpp::Logger BUFFER_LOGGER = rclcpp::get_logger("scene_buffer");

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

bool SceneBuffer::init()
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
  return true;
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
  robot_model_loader::RobotModelLoader rml(
    node_, robot_model_loader::RobotModelLoader::Options(urdf, srdf));
  model = rml.getModel();
  state = std::make_shared<moveit::core::RobotState>(model);

  state->setToDefaultValues();
  model->printModelInfo(std::cout);
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
  obstacles.header.frame_id = model->getModelFrame();
  obstacles.meshes.reserve(mesh_links.size() + 10); // 10 for attached objects
  obstacles.primitives.reserve(prim_links.size() + 10);
  obstacles.meshes_poses.reserve(mesh_links.size() + 10);
  obstacles.primitives_poses.reserve(prim_links.size() + 10);
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
      marker.header.frame_id = obs.header.frame_id;
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
  for(size_t i=links.size(); i < obs.meshes.size(); i++)
  {
    const auto& mesh_tri = obs.meshes[i].triangles;
    const auto& mesh_ver = obs.meshes[i].vertices;
    for(size_t id=0; id < obs.meshes_poses[i].poses.size(); id++)
    {
      if(id % step != 0 && id != obs.meshes_poses[i].poses.size() - 1){
        continue;
      }
      const auto& pose = obs.meshes_poses[i].poses[id];
      visualization_msgs::msg::Marker marker;
      marker.header.stamp = time_now;
      marker.header.frame_id = obs.header.frame_id;
      marker.ns = "attached";
      marker.id = id;
      marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.lifetime = rclcpp::Duration(std::chrono::seconds(3));
      marker.frame_locked = false;
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
      for(const auto& idx : mesh_tri){
        for(size_t j=0; j<3; j++){
          marker.points.push_back(mesh_ver[idx.vertex_indices[j]]);
        }
      }
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

  const auto& get_model_poses_from_state = [&eigen_to_msg, &pose_adjust](
    const std::shared_ptr<Robot>& robot, std::vector<Eigen::Isometry3d>& last_trans){
      const size_t mesh_links_size = robot->mesh_links.size();
      const size_t prim_links_size = robot->prim_links.size();
      const size_t mesh_attac_size = robot->attached_mesh_obj_idx.size();
      const size_t prim_attac_size = robot->attached_prim_obj_idx.size();
      const bool check_diff = last_trans.size() != 0;
      if(last_trans.size() == 0){
        last_trans.resize(mesh_links_size + prim_links_size + mesh_attac_size + prim_attac_size);
      }
      assert(last_trans.size() == mesh_links_size + prim_links_size + mesh_attac_size + prim_attac_size);
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
      for(const auto& [name, idx] : robot->attached_mesh_obj_idx){
        trans_list.clear();
        trans_list.emplace_back(const_state->getAttachedBody(name)->getGlobalPose());
        if(check_diff && !pose_adjust(trans_list, last_trans[idx + prim_links_size])){
          continue;
        }
        for(const auto& trans : trans_list){
          robot->obstacles.meshes_poses[idx].poses.push_back(eigen_to_msg(trans));
        }
        last_trans[idx + prim_links_size] = trans_list[0];
      }
      for(const auto& [name, idx] : robot->attached_prim_obj_idx){
        trans_list.clear();
        trans_list.emplace_back(const_state->getAttachedBody(name)->getGlobalPose());
        if(check_diff && !pose_adjust(trans_list, last_trans[idx + mesh_links_size + mesh_attac_size])){
          continue;
        }
        for(const auto& trans : trans_list){
          robot->obstacles.primitives_poses[idx].poses.push_back(eigen_to_msg(trans));
        }
        last_trans[idx + mesh_links_size + mesh_attac_size] = trans_list[0];
      }
    };

  const auto& check_running_time_with_delay = 
    [this](const rclcpp::Time& start_time, const rclcpp::Time& tarj_start_time, const auto& point){
      return start_time > (tarj_start_time + 
        rclcpp::Duration(point.time_from_start) + *delay_duration_);
    };

  const auto& get_pose_from_trajectory = 
    [&check_running_time_with_delay, &get_model_poses_from_state](
      const auto& robot, const auto&  trajectory, const auto& start_time, bool running_traj){
        if(!trajectory){
          return;
        }
        std::vector<Eigen::Isometry3d> last_trans;
        const auto& traj_joint_names = trajectory->joint_names;
        const auto& tarj_start_time = rclcpp::Time(trajectory->header.stamp);
        for(const auto& point : trajectory->points)
        {
          // always keep last ponits for running trajectory
          if(running_traj && point != trajectory->points.back() &&
            check_running_time_with_delay(start_time, tarj_start_time, point)){
            continue;
          }
          for(size_t i=0; i<traj_joint_names.size(); i++)
          {
            robot->state->setJointPositions(
              traj_joint_names[i], &(point.positions[i]));
          }
          robot->state->update();
          get_model_poses_from_state(robot, last_trans);
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

  const auto try_to_set_some_one_is_planning = [this](const std::string& name, std::string& planning_name){
    const std::lock_guard<std::mutex> planning_lock(some_one_is_planning_mutex_);
    if(some_one_is_planning_ == "" || some_one_is_planning_ == name){
      some_one_is_planning_ = name;
      return true;
    }else{
      planning_name = some_one_is_planning_;
      return false;
    }
  };

  std::string now_planning("");
  while(!try_to_set_some_one_is_planning(req_robot_name, now_planning)){
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    RCLCPP_INFO(get_logger(), 
      "%s Waiting for %s planning", req_robot_name.c_str(), now_planning.c_str());
  }

  const rclcpp::Time start_time = [this, &req_robot_name]{
    const rclcpp::Time& now = this->now();
    const auto& traj = robots_.at(req_robot_name)->running_trajectory;
    if(!traj){
      return now;
    }
    const rclcpp::Time& traj_end = rclcpp::Time(traj->header.stamp) + 
      rclcpp::Duration(traj->points.back().time_from_start);
    return (traj_end > now) ? traj_end : now;
  }();

  const auto& collision_robots =
    params_.collision_maps.robot_names_map.at(req_robot_name).collision_robots;

  res->obstacles_list.reserve(collision_robots.size());

  for(const auto& other_name : collision_robots)
  {
    if(other_name=="empty")
      continue;
    const auto& other_robot = robots_.at(other_name);
    other_robot->clean_poses();
    const std::lock_guard<std::shared_mutex> state_lock(other_robot->get_state_data_mutex());
    const std::shared_lock<std::shared_mutex> traj_lock(other_robot->get_trajectory_data_mutex());

    if(!(other_robot->planned_trajectory || other_robot->running_trajectory))
    {
      other_robot->update_to_current();
      std::vector<Eigen::Isometry3d> last_trans;
      get_model_poses_from_state(other_robot, last_trans);
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
      if(other_name=="empty")
        continue;
      pub_obstacles(*robots_.at(other_name), params_.obstacle_pub_step);
    }
  }
  rclcpp::Time t2 = this->now();
  std::cout<<"get_obstacle_cb end time = "<<(t2 - t1).seconds()<<std::endl;
  return true;
}

bool SceneBuffer::Robot::obstacles_from_links()
{
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

void SceneBuffer::Robot::attachObjectCallback(const moveit_msgs::msg::AttachedCollisionObject::ConstSharedPtr& obj_msg)
{
// bool SceneBuffer::Robot::process_attached_collision_object_msg(const moveit_msgs::msg::AttachedCollisionObject& obj_msg)
  if (obj_msg->object.operation == moveit_msgs::msg::CollisionObject::ADD &&
      !model->hasLinkModel(obj_msg->link_name))
  {
    RCLCPP_ERROR(ROBOT_LOGGER, "Unable to attach a body to link '%s' (link not found)", obj_msg->link_name.c_str());
    return;
  }

  if (obj_msg->object.id == "<octomap>")
  {
    RCLCPP_ERROR(ROBOT_LOGGER, "The ID '%s' cannot be used for collision objects (name reserved)", "<octomap>");
    return;
  }

  if (!state)  // there must be a parent in this case
  {
    RCLCPP_ERROR(ROBOT_LOGGER, "No robot state");
    return;
  }
  const std::lock_guard<std::shared_mutex> data_lock(state_data_mutex_);
  state->update();

  // The ADD/REMOVE operations follow this order:
  // STEP 1: Get info about the object from either the message or the world/RobotState
  // STEP 2: Remove the object from the world/RobotState if necessary
  // STEP 3: Put the object in the RobotState/world

  if (obj_msg->object.operation == moveit_msgs::msg::CollisionObject::ADD ||
      obj_msg->object.operation == moveit_msgs::msg::CollisionObject::APPEND)
  {
    const moveit::core::LinkModel* link_model = model->getLinkModel(obj_msg->link_name);
    if (link_model)
    {
      // items to build the attached object from (filled from existing world object or message)
      Eigen::Isometry3d object_pose_in_link;
      std::vector<shapes::ShapeConstPtr> shapes;
      EigenSTL::vector_Isometry3d shape_poses;
      moveit::core::FixedTransformsMap subframe_poses;

      // STEP 1: Obtain info about object to be attached.
      //         If it is in the world, message contents are ignored.


      Eigen::Isometry3d header_frame_to_object_pose;
      if (!shapesAndPosesFromCollisionObjectMessage(obj_msg->object, header_frame_to_object_pose, shapes, shape_poses))
        return;
      const Eigen::Isometry3d world_to_header_frame = getFrameTransform(obj_msg->object.header.frame_id);
      const Eigen::Isometry3d link_to_header_frame =
          state->getGlobalLinkTransform(link_model).inverse() * world_to_header_frame;
      object_pose_in_link = link_to_header_frame * header_frame_to_object_pose;

      Eigen::Isometry3d subframe_pose;
      for (std::size_t i = 0; i < obj_msg->object.subframe_poses.size(); ++i)
      {
        poseMsgToEigen(obj_msg->object.subframe_poses[i], subframe_pose);
        std::string name = obj_msg->object.subframe_names[i];
        subframe_poses[name] = subframe_pose;
      }


      if (shapes.empty())
      {
        RCLCPP_ERROR(ROBOT_LOGGER, "There is no geometry to attach to link '%s' as part of attached body '%s'",
                     obj_msg->link_name.c_str(), obj_msg->object.id.c_str());
        return;
      }

      // STEP 3: Attach the object to the robot
      if (obj_msg->object.operation == moveit_msgs::msg::CollisionObject::ADD ||
          !state->hasAttachedBody(obj_msg->object.id))
      {
        if (state->clearAttachedBody(obj_msg->object.id))
        {
          RCLCPP_INFO(ROBOT_LOGGER,
                       "The robot state already had an object named '%s' attached to link '%s'. "
                       "The object was replaced.",
                       obj_msg->object.id.c_str(), obj_msg->link_name.c_str());
        }
        state->attachBody(obj_msg->object.id, object_pose_in_link, shapes, shape_poses, obj_msg->touch_links,
                                 obj_msg->link_name, obj_msg->detach_posture, subframe_poses);
        
        if (obj_msg->object.meshes.size() > 0)
        {
          mr_msgs::msg::Poses p;
          attached_mesh_obj_idx.insert(std::pair<std::string, size_t>(
            obj_msg->object.id, obstacles.meshes.size()));
          obstacles.meshes.push_back(obj_msg->object.meshes[0]);
          obstacles.meshes_poses.emplace_back(std::move(p));
          if (obj_msg->object.meshes.size() > 1){
            RCLCPP_WARN(ROBOT_LOGGER, "Only support one shape in one attached object msg, the rest are ignored.");
          }
        }
        if (obj_msg->object.primitives.size() > 0)
        {
          mr_msgs::msg::Poses p;
          attached_prim_obj_idx.insert(std::pair<std::string, size_t>(
            obj_msg->object.id, obstacles.primitives.size()));
          obstacles.primitives.push_back(obj_msg->object.primitives[0]);
          obstacles.primitives_poses.emplace_back(std::move(p));
          if (obj_msg->object.primitives.size() > 1){
            RCLCPP_WARN(ROBOT_LOGGER, "Only support one shape in one attached object msg, the rest are ignored.");
          }
        }
        if (obj_msg->object.planes.size())
        {
          RCLCPP_WARN(ROBOT_LOGGER, "Planes object not supported.");
        }
        RCLCPP_INFO(ROBOT_LOGGER, "Attached object '%s' to link '%s'", obj_msg->object.id.c_str(), obj_msg->link_name.c_str());
      }
      else  // APPEND: augment to existing attached object
      {
        const moveit::core::AttachedBody* ab = state->getAttachedBody(obj_msg->object.id);
        RCLCPP_WARN(ROBOT_LOGGER, "Append not supported.");
        // Allow overriding the body's pose if provided, otherwise keep the old one
        if (isEmpty(obj_msg->object.pose))
          object_pose_in_link = ab->getPose();  // Keep old pose

        shapes.insert(shapes.end(), ab->getShapes().begin(), ab->getShapes().end());
        shape_poses.insert(shape_poses.end(), ab->getShapePoses().begin(), ab->getShapePoses().end());
        subframe_poses.insert(ab->getSubframes().begin(), ab->getSubframes().end());
        trajectory_msgs::msg::JointTrajectory detach_posture =
            obj_msg->detach_posture.joint_names.empty() ? ab->getDetachPosture() : obj_msg->detach_posture;

        std::set<std::string> touch_links = ab->getTouchLinks();
        touch_links.insert(std::make_move_iterator(obj_msg->touch_links.begin()),
                           std::make_move_iterator(obj_msg->touch_links.end()));

        state->clearAttachedBody(obj_msg->object.id);
        state->attachBody(obj_msg->object.id, object_pose_in_link, shapes, shape_poses, touch_links,
                                 obj_msg->link_name, detach_posture, subframe_poses);
        RCLCPP_INFO(ROBOT_LOGGER, "Appended things to object '%s' attached to link '%s'", obj_msg->object.id.c_str(),
                     obj_msg->link_name.c_str());
      }
      return;
    }
    else
    {
      RCLCPP_ERROR(ROBOT_LOGGER, "Robot state is not compatible with robot model. This could be fatal.");
    }
  }
  else if (obj_msg->object.operation == moveit_msgs::msg::CollisionObject::REMOVE)  // == DETACH
  {
    // STEP 1: Get info about the object from the RobotState
    std::vector<const moveit::core::AttachedBody*> attached_bodies;
    if (obj_msg->object.id.empty())
    {
      const moveit::core::LinkModel* link_model =
          obj_msg->link_name.empty() ? nullptr : model->getLinkModel(obj_msg->link_name);
      if (link_model)
      {  // if we have a link model specified, only fetch bodies attached to this link
        state->getAttachedBodies(attached_bodies, link_model);
      }
      else
      {
        state->getAttachedBodies(attached_bodies);
      }
    }
    else  // A specific object id will be removed.
    {
      const moveit::core::AttachedBody* body = state->getAttachedBody(obj_msg->object.id);
      if (body)
      {
        if (!obj_msg->link_name.empty() && (body->getAttachedLinkName() != obj_msg->link_name))
        {
          RCLCPP_ERROR_STREAM(ROBOT_LOGGER, "The AttachedCollisionObject message states the object is attached to "
                                          << obj_msg->link_name << ", but it is actually attached to "
                                          << body->getAttachedLinkName()
                                          << ". Leave the link_name empty or specify the correct link.");
          return;
        }
        attached_bodies.push_back(body);
      }
    }

    // STEP 2+3: Remove the attached object(s) from the RobotState and put them in the world
    for (const moveit::core::AttachedBody* attached_body : attached_bodies)
    {
      const std::string& name = attached_body->getName();
      if (attached_mesh_obj_idx.find(name) != attached_mesh_obj_idx.end()){
        const size_t idx = attached_mesh_obj_idx.at(name);
        attached_mesh_obj_idx.erase(name);
        obstacles.meshes.erase(obstacles.meshes.begin() + idx);
        obstacles.meshes_poses.erase(obstacles.meshes_poses.begin() + idx);
        for (auto& it : attached_mesh_obj_idx){
          if (it.second > idx){
            it.second -= 1;
          }
        }
      } else if (attached_prim_obj_idx.find(name) != attached_prim_obj_idx.end()){
        const size_t idx = attached_prim_obj_idx.at(name);
        attached_prim_obj_idx.erase(name);
        obstacles.primitives.erase(obstacles.primitives.begin() + idx);
        obstacles.primitives_poses.erase(obstacles.primitives_poses.begin() + idx);
        for (auto& it : attached_prim_obj_idx){
          if (it.second > idx){
            it.second -= 1;
          }
        }
      }
      state->clearAttachedBody(name);
    }
    if (!attached_bodies.empty() || obj_msg->object.id.empty())
      return;
  }
  else if (obj_msg->object.operation == moveit_msgs::msg::CollisionObject::MOVE)
  {
    RCLCPP_ERROR(ROBOT_LOGGER, "Move for attached objects not yet implemented");
  }
  else
  {
    RCLCPP_ERROR(ROBOT_LOGGER, "Unknown collision object operation: %d", obj_msg->object.operation);
  }

  return;
}

bool SceneBuffer::Robot::shapesAndPosesFromCollisionObjectMessage(const moveit_msgs::msg::CollisionObject& object,
                                                                  Eigen::Isometry3d& object_pose,
                                                                  std::vector<shapes::ShapeConstPtr>& shapes,
                                                                  EigenSTL::vector_Isometry3d& shape_poses)
{
  if (object.primitives.size() < object.primitive_poses.size())
  {
    RCLCPP_ERROR(ROBOT_LOGGER, "More primitive shape poses than shapes in collision object message.");
    return false;
  }
  if (object.meshes.size() < object.mesh_poses.size())
  {
    RCLCPP_ERROR(ROBOT_LOGGER, "More mesh poses than meshes in collision object message.");
    return false;
  }
  if (object.planes.size() < object.plane_poses.size())
  {
    RCLCPP_ERROR(ROBOT_LOGGER, "More plane poses than planes in collision object message.");
    return false;
  }

  const int num_shapes = object.primitives.size() + object.meshes.size() + object.planes.size();
  shapes.reserve(num_shapes);
  shape_poses.reserve(num_shapes);

  poseMsgToEigen(object.pose, object_pose);

  bool switch_object_pose_and_shape_pose = false;
  if (num_shapes == 1)
  {
    if (isEmpty(object.pose))
    {
      switch_object_pose_and_shape_pose = true;  // If the object pose is not set but the shape pose is,
                                                 // use the shape's pose as the object pose.
    }
  }

  auto append = [this, &object_pose, &shapes, &shape_poses,
                 &switch_object_pose_and_shape_pose](shapes::Shape* s, const geometry_msgs::msg::Pose& pose_msg) {
    if (!s)
      return;
    Eigen::Isometry3d pose;
    poseMsgToEigen(pose_msg, pose);
    if (!switch_object_pose_and_shape_pose)
    {
      shape_poses.emplace_back(std::move(pose));
    }
    else
    {
      shape_poses.emplace_back(std::move(object_pose));
      object_pose = pose;
    }
    shapes.emplace_back(shapes::ShapeConstPtr(s));
  };

  auto treat_shape_vectors = [this, &append](const auto& shape_vector,        // the shape_msgs of each type
                                       const auto& shape_poses_vector,  // std::vector<const geometry_msgs::Pose>
                                       const std::string& shape_type) {
    if (shape_vector.size() > shape_poses_vector.size())
    {
      RCLCPP_DEBUG_STREAM(ROBOT_LOGGER, "Number of " << shape_type
                                               << " does not match number of poses "
                                                  "in collision object message. Assuming identity.");
      for (std::size_t i = 0; i < shape_vector.size(); ++i)
      {
        if (i >= shape_poses_vector.size())
        {
          append(shapes::constructShapeFromMsg(shape_vector[i]),
                 geometry_msgs::msg::Pose());  // Empty shape pose => Identity
        }
        else
          append(shapes::constructShapeFromMsg(shape_vector[i]), shape_poses_vector[i]);
      }
    }
    else
    {
      for (std::size_t i = 0; i < shape_vector.size(); ++i)
        append(shapes::constructShapeFromMsg(shape_vector[i]), shape_poses_vector[i]);
    }
  };

  treat_shape_vectors(object.primitives, object.primitive_poses, std::string("primitive_poses"));
  treat_shape_vectors(object.meshes, object.mesh_poses, std::string("meshes"));
  treat_shape_vectors(object.planes, object.plane_poses, std::string("planes"));
  return true;
}

void SceneBuffer::Robot::poseMsgToEigen(const geometry_msgs::msg::Pose& msg, Eigen::Isometry3d& out)
{
  Eigen::Translation3d translation(msg.position.x, msg.position.y, msg.position.z);
  Eigen::Quaterniond quaternion(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);
  quaternion.normalize();
  out = translation * quaternion;
}

const Eigen::Isometry3d& SceneBuffer::Robot::getFrameTransform(const std::string& frame_id) const
{
  if (!frame_id.empty() && frame_id[0] == '/')
  {
    // Recursively call itself without the slash in front of frame name
    return getFrameTransform(frame_id.substr(1));
  }

  bool frame_found;
  const Eigen::Isometry3d& t1 = state->getFrameTransform(frame_id, &frame_found);
  if (frame_found)
    return t1;
  else
  {
    RCLCPP_ERROR(ROBOT_LOGGER, "Frame not found when trying to get frame transfrom.");
    throw;
  }
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