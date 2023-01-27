#include "scene_buffer/scene_buffer.hpp"

SceneBuffer::SceneBuffer(const std::string& node_name, const rclcpp::NodeOptions& node_options)
: Node(node_name, node_options)
{
  get_obstacle_service_ = this->create_service<example_interfaces::srv::AddTwoInts>(
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
    std::cout<<"eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee"<<std::endl;
    robot_states_[robot_name]->printTransforms();
    ang = 0.1;
    robot_states_[robot_name]->setJointPositions("shoulder_lift_joint", &ang);
    robot_states_[robot_name]->update();
    std::cout<<"ffffffffffffffffffffffffffffffffffffffffffffffffffff"<<std::endl;
    robot_states_[robot_name]->printTransforms();
  }
}

void SceneBuffer::get_obstacle_cb(const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> req,
  std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> res)
{
  return;
}