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
    robot_models_.insert(std::pair<
      std::string, moveit::core::RobotModelPtr>(robot_name, rml.getModel()));
    robot_states_.insert(std::pair<std::string, moveit::core::RobotStatePtr>(
      robot_name, new moveit::core::RobotState(robot_models_[robot_name])));
    robot_states_[robot_name]->setToDefaultValues();
    std::cout<<"ccccccccccccccccccccccccccccccccccccccccccccccccccc"<<std::endl;
    robot_models_[robot_name]->printModelInfo(std::cout);
    std::cout<<"ddddddddddddddddddddddddddddddddddddddddddddddddddd"<<std::endl;
    robot_states_[robot_name]->printTransforms();
    double ang = -0.1;
    robot_states_[robot_name]->setJointPositions("shoulder_lift_joint", &ang);
    robot_states_[robot_name]->update();
    {
      std::cout<<"BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB"<<std::endl;
      const auto links = robot_models_[robot_name]->getLinkModels();
      for(const auto link : links)
      {
        const auto shap = link->getShapes();
        const auto trans = robot_states_[robot_name]->getGlobalLinkTransform(link);
        Eigen::Matrix3d m = trans.rotation();
        Eigen::Vector3d v = trans.translation();
        std::cout << "Rotation: " << std::endl << m << std::endl;
        std::cout << "Translation: " << std::endl << v << std::endl;
        link_poses.push_back(trans);
      }
      link_poses_.push_back(link_poses);
      link_poses.clear();
      std::cout<<"BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB"<<std::endl;
    }
    std::cout<<"eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee"<<std::endl;
    robot_states_[robot_name]->printTransforms();
    ang = 0.1;
    robot_states_[robot_name]->setJointPositions("shoulder_lift_joint", &ang);
    robot_states_[robot_name]->update();
    std::cout<<"ffffffffffffffffffffffffffffffffffffffffffffffffffff"<<std::endl;
    robot_states_[robot_name]->printTransforms();

    {
      std::cout<<"DDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDD"<<std::endl;
      const auto links = robot_models_[robot_name]->getLinkModels();
      for(const auto link : links)
      {
        const auto shap = link->getShapes();
        std::cout << "shap size = " << shap.size() << std::endl;
        const auto trans = robot_states_[robot_name]->getGlobalLinkTransform(link);
        Eigen::Matrix3d m = trans.rotation();
        Eigen::Vector3d v = trans.translation();
        std::cout << "Rotation: " << std::endl << m << std::endl;
        std::cout << "Translation: " << std::endl << v << std::endl;
        link_poses.push_back(trans);
      }
      link_poses_.push_back(link_poses);
      link_poses.clear();
      std::cout<<"DDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDD"<<std::endl;
    }
    std::cout<<"size of link_poses = "<<link_poses_.size()<<" x "<<link_poses_[0].size()<<std::endl;
  }
}

void SceneBuffer::get_obstacle_cb(const std::shared_ptr<ObstacleSrv::Request> req,
  std::shared_ptr<ObstacleSrv::Response> res)
{
  if(collision_map_.find(req->robot_name) == collision_map_.end()){
    RCLCPP_ERROR(get_logger(), "Request robot is not in the collision map");
    return;
  }
  const rclcpp::Time start_time(
    rclcpp::Time(req->header.stamp) + rclcpp::Duration(req->run_after));

  const auto eigen_to_msg = [](const Eigen::Isometry3d& trans){
    const Eigen::Quaterniond q(trans.rotation());
    const Eigen::Vector3d v(trans.translation());
    geometry_msgs::msg::Pose p;
    p.position.x = v(0);
    p.position.y = v(1);
    p.position.z = v(2);
    p.orientation.w = q.w();
    p.orientation.x = q.x();
    p.orientation.y = q.y();
    p.orientation.z = q.z();
    return p;
  };

  for(const auto& other_name : collision_map_[req->robot_name])
  {
    const rclcpp::Time other_start_time(
      trajectories_[other_name]->header.stamp);
    const auto traj_joint_names = trajectories_[other_name]->joint_names;

    mr_msgs::msg::Obstacles obstacles;
    const auto links = robot_models_[other_name]->getLinkModels();
    for(const auto link : links)
    {
      const auto shape = link->getShapes();
      if(shape.at(0)->type == shapes::MESH)
      {
        mesh_msg_from_shape(shape, obstacles.meshes);
      }
      else if(shape.at(0)->type == shapes::BOX || shape.at(0)->type == shapes::CONE || 
        shape.at(0)->type == shapes::CYLINDER || shape.at(0)->type == shapes::SPHERE)
      {

      }
    }

    for(const auto& point : trajectories_[other_name]->points)
    {
      if(start_time > (other_start_time + rclcpp::Duration(point.time_from_start))){
        continue;
      }
      for(size_t i=0; i<traj_joint_names.size(); i++)
      {
        robot_states_[other_name]->setJointPositions(
          traj_joint_names[i], &(point.positions[i]));
      }
      robot_states_[other_name]->update();
      for(const auto link : links)
      {
        const auto trans = robot_states_[other_name]->getGlobalLinkTransform(link);
        const geometry_msgs::msg::Pose p = eigen_to_msg(trans);
        obstacles.mesh_poses.push_back(p);
        res->dynamic_obstacles.push_back(obstacles);
      }
    }
  }
  return;
}

bool SceneBuffer::mesh_msg_from_shape(const std::vector<shapes::ShapeConstPtr>& shapes,
                                      std::vector<shape_msgs::msg::Mesh> object_pose)
{
  // if (object.primitives.size() < object.primitive_poses.size())
  // {
  //   RCLCPP_ERROR(LOGGER, "More primitive shape poses than shapes in collision object message.");
  //   return false;
  // }
  // if (object.meshes.size() < object.mesh_poses.size())
  // {
  //   RCLCPP_ERROR(LOGGER, "More mesh poses than meshes in collision object message.");
  //   return false;
  // }
  // if (object.planes.size() < object.plane_poses.size())
  // {
  //   RCLCPP_ERROR(LOGGER, "More plane poses than planes in collision object message.");
  //   return false;
  // }

  // const int num_shapes = object.primitives.size() + object.meshes.size() + object.planes.size();
  // shapes.reserve(num_shapes);
  // shape_poses.reserve(num_shapes);

  // PlanningScene::poseMsgToEigen(object.pose, object_pose);

  // bool switch_object_pose_and_shape_pose = false;
  // if (num_shapes == 1)
  //   if (moveit::core::isEmpty(object.pose))
  //   {
  //     switch_object_pose_and_shape_pose = true;  // If the object pose is not set but the shape pose is,
  //                                                // use the shape's pose as the object pose.
  //   }

  // auto append = [&object_pose, &shapes, &shape_poses,
  //                &switch_object_pose_and_shape_pose](shapes::Shape* s, const geometry_msgs::msg::Pose& pose_msg) {
  //   if (!s)
  //     return;
  //   Eigen::Isometry3d pose;
  //   PlanningScene::poseMsgToEigen(pose_msg, pose);
  //   if (!switch_object_pose_and_shape_pose)
  //     shape_poses.emplace_back(std::move(pose));
  //   else
  //   {
  //     shape_poses.emplace_back(std::move(object_pose));
  //     object_pose = pose;
  //   }
  //   shapes.emplace_back(shapes::ShapeConstPtr(s));
  // };

  // auto treat_shape_vectors = [&append](const auto& shape_vector,        // the shape_msgs of each type
  //                                      const auto& shape_poses_vector,  // std::vector<const geometry_msgs::Pose>
  //                                      const std::string& shape_type) {
  //   if (shape_vector.size() > shape_poses_vector.size())
  //   {
  //     RCLCPP_DEBUG_STREAM(LOGGER, "Number of " << shape_type
  //                                              << " does not match number of poses "
  //                                                 "in collision object message. Assuming identity.");
  //     for (std::size_t i = 0; i < shape_vector.size(); ++i)
  //     {
  //       if (i >= shape_poses_vector.size())
  //       {
  //         append(shapes::constructShapeFromMsg(shape_vector[i]),
  //                geometry_msgs::msg::Pose());  // Empty shape pose => Identity
  //       }
  //       else
  //         append(shapes::constructShapeFromMsg(shape_vector[i]), shape_poses_vector[i]);
  //     }
  //   }
  //   else
  //     for (std::size_t i = 0; i < shape_vector.size(); ++i)
  //       append(shapes::constructShapeFromMsg(shape_vector[i]), shape_poses_vector[i]);
  // };

  // treat_shape_vectors(object.primitives, object.primitive_poses, std::string("primitive_poses"));
  // treat_shape_vectors(object.meshes, object.mesh_poses, std::string("meshes"));
  // treat_shape_vectors(object.planes, object.plane_poses, std::string("planes"));
  return true;
}