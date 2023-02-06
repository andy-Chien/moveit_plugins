#include <Eigen/Geometry>
#include <geometric_shapes/shape_operations.h>

#include "scene_buffer/scene_buffer.hpp"

SceneBuffer::SceneBuffer(const std::string& node_name, const rclcpp::NodeOptions& node_options)
: Node(node_name, node_options)
{
  get_obstacle_service_ = this->create_service<ObstacleSrv>(
    "get_trajectory_obstacle", std::bind(
      &SceneBuffer::get_obstacle_cb, this, std::placeholders::_1, std::placeholders::_2));  
}

void SceneBuffer::load_robots(const std::vector<std::string>& robot_names)
{
  for(auto robot_name : robot_names)
  {
    // robot_model_loader::RobotModelLoader rml(
    //   shared_from_this(), robot_name + "/robot_description");
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    const auto param_client_node = std::make_shared<rclcpp::Node>(
      "param_client_node", robot_name); //, node_options);
    const auto param_client = std::make_shared<rclcpp::SyncParametersClient>(
      param_client_node, "move_group");

    auto robot = std::make_shared<Robot>();
    
    const std::string urdf_string = 
      param_client->get_parameter<std::string>("robot_description");
    const std::string srdf_string = 
      param_client->get_parameter<std::string>("robot_description_semantic");
    // const rclcpp::Parameter kinematics_param = 
    //   param_client->get_parameter<rclcpp::Parameter>("robot_description_kinematics");

    // param_client_node->declare_parameter<rclcpp::Parameter>(kinematics_param);

    std::cout<<urdf_string<<std::endl;
    std::cout<<"XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX"<<std::endl;
    std::cout<<srdf_string<<std::endl;

    std::vector<Eigen::Isometry3d> link_poses;
    std::vector<std::vector<Eigen::Isometry3d>> link_poses_;

    robot_model_loader::RobotModelLoader rml(
      param_client_node, robot_model_loader::RobotModelLoader::Options(urdf_string, srdf_string));
    robot->model = rml.getModel();
    robot->state = std::make_shared<moveit::core::RobotState>(robot->model);

    robot->state->setToDefaultValues();
    std::cout<<"ccccccccccccccccccccccccccccccccccccccccccccccccccc"<<std::endl;
    robot->model->printModelInfo(std::cout);
    std::cout<<"ddddddddddddddddddddddddddddddddddddddddddddddddddd"<<std::endl;
    robot->state->printTransforms();
    double ang = -0.1;
    robot->state->setJointPositions("shoulder_lift_joint", &ang);
    robot->state->update();
    {
      std::cout<<"BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB"<<std::endl;
      const auto& links = robot->model->getLinkModels();
      for(const auto& link : links)
      {
        const auto& shap = link->getShapes();
        std::cout << "shap size = " << shap.size() << std::endl;
        const auto& trans = robot->state->getGlobalLinkTransform(link);
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
    robot->state->printTransforms();
    ang = 0.1;
    robot->state->setJointPositions("shoulder_lift_joint", &ang);
    robot->state->update();
    std::cout<<"ffffffffffffffffffffffffffffffffffffffffffffffffffff"<<std::endl;
    robot->state->printTransforms();

    {
      std::cout<<"DDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDD"<<std::endl;
      const auto links = robot->model->getLinkModels();
      for(const auto link : links)
      {
        const auto& shap = link->getShapes();
        std::cout << "shap size = " << shap.size() << std::endl;
        const auto& trans = robot->state->getGlobalLinkTransform(link);
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

    if(!obstacles_from_links(robot->model->getLinkModels(), robot->obstacles)){
      RCLCPP_ERROR(get_logger(), 
        "Convert from link to obstcale msg failed");
    }

    robots_.insert(std::pair<std::string, std::shared_ptr<Robot>>(
      robot_name, robot)
    );
  }
}

void SceneBuffer::get_obstacle_cb(const std::shared_ptr<ObstacleSrv::Request> req,
  std::shared_ptr<ObstacleSrv::Response> res)
{
  if(robots_.find(req->robot_name) == robots_.end()){
    RCLCPP_ERROR(get_logger(), "Requested robot has not been registered");
    return;
  }
  const rclcpp::Time start_time(
    rclcpp::Time(req->header.stamp) + rclcpp::Duration(req->run_after));

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
  const auto& robot = robots_.at(req->robot_name);
  for(const auto& other_name : robot->collision_map)
  {
    const auto& other_robot = robots_.at(other_name);
    if(!other_robot->trajectory){
      continue;
    }

    const rclcpp::Time other_start_time(other_robot->trajectory->header.stamp);
    const auto& traj_joint_names = other_robot->trajectory->joint_names;
    const auto& links = robot->model->getLinkModels();

    mr_msgs::msg::Obstacles obstacles;
    obstacles.meshes_poses.resize(links.size());

    for(const auto& point : other_robot->trajectory->points)
    {
      if(start_time > (other_start_time + rclcpp::Duration(point.time_from_start))){
        continue;
      }
      for(size_t i=0; i<traj_joint_names.size(); i++)
      {
        other_robot->state->setJointPositions(
          traj_joint_names[i], &(point.positions[i]));
      }
      other_robot->state->update();

      for(size_t i=0; i<links.size(); i++)
      {
        const auto& link = links[i];
        const auto& trans = other_robot->state->getGlobalLinkTransform(link);
        obstacles.meshes_poses[i].poses.push_back(eigen_to_msg(trans));
      }
    }
    res->dynamic_obstacles.push_back(obstacles);
  }
  return;
}

bool SceneBuffer::obstacles_from_links(
  const std::vector<moveit::core::LinkModel*> links, mr_msgs::msg::Obstacles& obstacles)
{
  obstacles.meshes.reserve(links.size());
  for(const auto& link : links)
  {
    if(link->getShapes().size() == 0){
      continue;
    }
    if(link->getShapes().size() > 1){
      RCLCPP_ERROR(get_logger(), 
        "Now it only support one shape for each link");
      return false;
    }
    const auto& shape = link->getShapes().at(0);
    if(shape->type == shapes::MESH)
    {
      shape_msgs::msg::Mesh mesh_msg;
      mesh_msg_from_shape(shape, mesh_msg);
      obstacles.meshes.push_back(mesh_msg);
    }
    else if(shape->type == shapes::BOX || shape->type == shapes::CONE || 
      shape->type == shapes::CYLINDER || shape->type == shapes::SPHERE)
    {

    }
  }
  return true;
}

bool SceneBuffer::mesh_msg_from_shape(const shapes::ShapeConstPtr shape,
                                      shape_msgs::msg::Mesh& mesh_msg)
{
  // PlanningScene::poseMsgToEigen(object.pose, object_pose);
  const shapes::Mesh* mesh = static_cast<const shapes::Mesh*>(shape.get());
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
  return true;
}