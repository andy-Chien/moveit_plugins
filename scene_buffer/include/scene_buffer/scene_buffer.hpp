#include <mutex>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include <geometric_shapes/check_isometry.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.h>
#include <memory>

#include "mr_msgs/msg/obstacles.hpp"
#include "mr_msgs/srv/get_robot_trajectory_obstacle.hpp"
#include "mr_msgs/srv/set_trajectory_state.hpp"
#include "scene_buffer_parameters.hpp"

class SceneBuffer : public rclcpp::Node
{
public:
  using TrajectoryMsg = trajectory_msgs::msg::JointTrajectory;

  SceneBuffer(const std::string& node_name, const rclcpp::NodeOptions& node_options);
  void init();

  class Robot
  {
  public:
    Robot(std::shared_ptr<rclcpp::Node> node, std::string robot_name, double padding)
    : node_(node), robot_name_(robot_name), padding_(padding)
    {
      obstacles.name = robot_name + "_trajectory_obstacles";
      load_robot(robot_name);

      jnt_states_sub_ = node->create_subscription<sensor_msgs::msg::JointState>(
        robot_name + "/joint_states", 1, 
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) -> void {
          if(jnt_names_.size() != msg->name.size()){
            jnt_names_ = msg->name;
          }
          jnt_pos_ = msg->position;
        }
      );
    }

    void update_to_current()
    {
      for(size_t i=0; i<jnt_names_.size(); i++){
        state->setJointPositions(jnt_names_.at(i), &(jnt_pos_[i]));
      }
      state->update();
    }

    void clean_poses()
    {
      for(auto& x : obstacles.meshes_poses){
        x.poses.clear();
      }
      for(auto& x : obstacles.primitives_poses){
        x.poses.clear();
      }
    }

    moveit::core::RobotModelPtr model;
    moveit::core::RobotStatePtr state;
    mr_msgs::msg::Obstacles obstacles;
    std::vector<moveit::core::LinkModel*> mesh_links;
    std::vector<moveit::core::LinkModel*> prim_links;
    std::vector<std::shared_ptr<TrajectoryMsg>> trajectories;
    std::shared_ptr<TrajectoryMsg> planned_trajectory{nullptr};
    std::shared_ptr<TrajectoryMsg> running_trajectory{nullptr};

  private:
    void load_robot(const std::string& robot_name);
    void load_robot(const std::string& urdf, const std::string& srdf);
    bool obstacles_from_links();

    std::shared_ptr<rclcpp::Node> node_;
    std::string robot_name_;
    double padding_;
    
    std::vector<double> jnt_pos_;
    std::vector<std::string> jnt_names_;
    rclcpp::AsyncParametersClient::SharedPtr param_client_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jnt_states_sub_;
  };

private:

  using ObstacleSrv = mr_msgs::srv::GetRobotTrajectoryObstacle;
  using TrajectorySrv = mr_msgs::srv::SetTrajectoryState;

  bool get_obstacle_cb(
    const std::shared_ptr<ObstacleSrv::Request> req,
    std::shared_ptr<ObstacleSrv::Response> res);

  bool set_trajectory_cb(
    const std::shared_ptr<TrajectorySrv::Request> req,
    std::shared_ptr<TrajectorySrv::Response> res);
  


  using Params = scene_buffer::Params;
  using ParamListener = scene_buffer::ParamListener;
  std::shared_ptr<ParamListener> param_listener_;
  Params params_;
  
  std::string some_one_is_planning_;
  std::mutex trajectory_data_mutex_;
  std::mutex some_one_is_planning_mutex_;
  rclcpp::CallbackGroup::SharedPtr cb_group_;

  std::unique_ptr<rclcpp::Duration> delay_duration_;
  std::map<std::string, std::shared_ptr<Robot>> robots_;
  rclcpp::Service<ObstacleSrv>::SharedPtr get_obstacle_service_;
  rclcpp::Service<TrajectorySrv>::SharedPtr set_trajectory_service_;
};

// // ==============================================================================
// // getXXX function in robot_model.h
// class robot_model_get
// {
//   const std::string& getName() const;
//   const std::string& getModelFrame() const;
//   const urdf::ModelInterfaceSharedPtr& getURDF() const;
//   const srdf::ModelConstSharedPtr& getSRDF() const;
//   const std::string& getRootJointName() const;
//   const std::vector<const JointModel*>& getJointModels() const;
//   const std::vector<JointModel*>& getJointModels();
//   const std::vector<std::string>& getJointModelNames() const;
//   const std::vector<const JointModel*>& getActiveJointModels() const;
//   const std::vector<std::string>& getActiveJointModelNames() const;
//   const std::vector<JointModel*>& getActiveJointModels();
//   const std::vector<const JointModel*>& getSingleDOFJointModels() const;
//   const std::vector<const JointModel*>& getMultiDOFJointModels() const;
//   const std::vector<const JointModel*>& getContinuousJointModels() const;
//   const std::vector<const JointModel*>& getMimicJointModels() const;
//   const std::string& getRootLinkName() const;
//   const std::vector<const LinkModel*>& getLinkModels() const;
//   const std::vector<LinkModel*>& getLinkModels();
//   const std::vector<std::string>& getLinkModelNames() const;
//   const std::vector<const LinkModel*>& getLinkModelsWithCollisionGeometry() const;
//   const std::vector<std::string>& getLinkModelNamesWithCollisionGeometry() const;
//   const std::vector<const JointModelGroup*>& getJointModelGroups() const;
//   const std::vector<JointModelGroup*>& getJointModelGroups();
//   const std::vector<std::string>& getJointModelGroupNames() const;
//   const std::vector<const JointModelGroup*>& getEndEffectors() const;
//   const std::vector<std::string>& getVariableNames() const;
//   const VariableBounds& getVariableBounds(const std::string& variable) const;
//   const JointBoundsVector& getActiveJointModelsBounds() const;
//   const JointModel* getRootJoint() const;;
//   const JointModel* getJointModel(const std::string& joint) const;;
//   const JointModel* getJointModel(size_t index) const;;
//   JointModel* getJointModel(const std::string& joint);;
//   const JointModel* getJointOfVariable(int variable_index) const;
//   const JointModel* getJointOfVariable(const std::string& variable) const;
//   const LinkModel* getRootLink() const;;
//   const LinkModel* getLinkModel(const std::string& link, bool* has_link = nullptr) const;;
//   const LinkModel* getLinkModel(size_t index) const;;
//   LinkModel* getLinkModel(const std::string& link, bool* has_link = nullptr);;
//   static const moveit::core::LinkModel* getRigidlyConnectedParentLinkModel(const LinkModel* link);;
//   const JointModelGroup* getJointModelGroup(const std::string& name) const;;
//   JointModelGroup* getJointModelGroup(const std::string& name);;
//   const JointModelGroup* getEndEffector(const std::string& name) const;;
//   JointModelGroup* getEndEffector(const std::string& name);;
//   const JointModel* getCommonRoot(const JointModel* a, const JointModel* b) const;
//   void getVariableRandomPositions(random_numbers::RandomNumberGenerator& rng, double* values) const;;
//   void getVariableDefaultPositions(double* values) const;;
//   void getVariableRandomPositions(random_numbers::RandomNumberGenerator& rng, std::vector<double>& values) const;;
//   void getVariableDefaultPositions(std::vector<double>& values) const;
//   void getVariableRandomPositions(random_numbers::RandomNumberGenerator& rng, std::map<std::string, double>& values) const;;
//   void getVariableDefaultPositions(std::map<std::string, double>& values) const;;
//   void getMissingVariableNames(const std::vector<std::string>& variables, std::vector<std::string>& missing_variables) const;;
//   std::size_t getJointModelCount() const;
//   std::size_t getLinkModelCount() const;
//   std::size_t getLinkGeometryCount() const;
//   std::size_t getVariableCount() const;
//   size_t getVariableIndex(const std::string& variable) const;;
// };
// // =============================================================================

// // =============================================================================
// // getXXX function in robot_state.h
// class robot_state_get
// {
//   const RobotModelConstPtr& getRobotModel() const;
//   const std::vector<std::string>& getVariableNames() const;
//   const Eigen::Isometry3d& getGlobalLinkTransform(const std::string& link_name);
//   const Eigen::Isometry3d& getGlobalLinkTransform(const LinkModel* link);
//   const Eigen::Isometry3d& getGlobalLinkTransform(const std::string& link_name) const;
//   const Eigen::Isometry3d& getGlobalLinkTransform(const LinkModel* link) const;
//   const Eigen::Isometry3d& getCollisionBodyTransform(const std::string& link_name, std::size_t index);
//   const Eigen::Isometry3d& getCollisionBodyTransform(const LinkModel* link, std::size_t index);
//   const Eigen::Isometry3d& getCollisionBodyTransform(const std::string& link_name, std::size_t index) const;
//   const Eigen::Isometry3d& getCollisionBodyTransform(const LinkModel* link, std::size_t index) const;
//   const Eigen::Isometry3d& getJointTransform(const std::string& joint_name);
//   const Eigen::Isometry3d& getJointTransform(const JointModel* joint);
//   const Eigen::Isometry3d& getJointTransform(const std::string& joint_name) const;
//   const Eigen::Isometry3d& getJointTransform(const JointModel* joint) const;
//   random_numbers::RandomNumberGenerator& getRandomNumberGenerator();
//   const Eigen::Isometry3d& getFrameTransform(const std::string& frame_id, bool* frame_found = nullptr);;
//   const Eigen::Isometry3d& getFrameTransform(const std::string& frame_id, bool* frame_found = nullptr) const;;
//   const Eigen::Isometry3d& getFrameInfo(const std::string& frame_id, const LinkModel*& robot_link, bool& frame_found) const;;
//   const LinkModel* getLinkModel(const std::string& link) const;
//   const JointModel* getJointModel(const std::string& joint) const;
//   const JointModelGroup* getJointModelGroup(const std::string& group) const;
//   double* getVariablePositions();
//   const double* getVariablePositions() const;
//   double* getVariableVelocities();
//   const double* getVariableVelocities() const;
//   double* getVariableAccelerations();
//   const double* getVariableAccelerations() const;
//   double* getVariableEffort();
//   const double* getVariableEffort() const;
//   const double* getJointPositions(const std::string& joint_name) const;
//   const double* getJointPositions(const JointModel* joint) const;
//   const double* getJointVelocities(const std::string& joint_name) const;
//   const double* getJointVelocities(const JointModel* joint) const;
//   const double* getJointAccelerations(const std::string& joint_name) const;
//   const double* getJointAccelerations(const JointModel* joint) const;
//   const double* getJointEffort(const std::string& joint_name) const;
//   const double* getJointEffort(const JointModel* joint) const;
//   const moveit::core::LinkModel* getRigidlyConnectedParentLinkModel(const std::string& frame) const;;
//   const AttachedBody* getAttachedBody(const std::string& name) const;;
//   void getAttachedBodies(std::vector<const AttachedBody*>& attached_bodies) const;;
//   void getAttachedBodies(std::vector<const AttachedBody*>& attached_bodies, const JointModelGroup* group) const;;
//   void getAttachedBodies(std::vector<const AttachedBody*>& attached_bodies, const LinkModel* link_model) const;;
//   void getRobotMarkers(visualization_msgs::msg::MarkerArray& arr, const std::vector<std::string>& link_names,
//     const std_msgs::msg::ColorRGBA& color, const std::string& ns, const rclcpp::Duration& dur, bool include_attached = false) const;;
//   void getRobotMarkers(visualization_msgs::msg::MarkerArray& arr, const std::vector<std::string>& link_names,
//     const std_msgs::msg::ColorRGBA& color, const std::string& ns, const rclcpp::Duration& dur, bool include_attached = false);
//   void getRobotMarkers(visualization_msgs::msg::MarkerArray& arr, const std::vector<std::string>& link_names, bool include_attached = false) const;;
//   void getRobotMarkers(visualization_msgs::msg::MarkerArray& arr, const std::vector<std::string>& link_names, bool include_attached = false);
//   void getMissingKeys(const std::map<std::string, double>& variable_map, std::vector<std::string>& missing_variables) const;;
//   void getStateTreeJointString(std::ostream& ss, const JointModel* jm, const std::string& pfx0, bool last) const;;
//   std::size_t getVariableCount() const;
// }
// // =============================================================================

// // =============================================================================
// // getXXX function in link_model.h
// class link_model_get
// {
//   const std::string& getName() const;
//   size_t getLinkIndex() const;
//   int getFirstCollisionBodyTransformIndex() const;
//   const JointModel* getParentJointModel() const;
//   const LinkModel* getParentLinkModel() const;
//   const std::vector<const JointModel*>& getChildJointModels() const;
//   const Eigen::Isometry3d& getJointOriginTransform() const;
//   const EigenSTL::vector_Isometry3d& getCollisionOriginTransforms() const;
//   const std::vector<shapes::ShapeConstPtr>& getShapes() const;
//   const Eigen::Vector3d& getShapeExtentsAtOrigin() const;
//   const Eigen::Vector3d& getCenteredBoundingBoxOffset() const;
//   const LinkTransformMap& getAssociatedFixedTransforms() const;
//   const std::string& getVisualMeshFilename() const;
//   const Eigen::Vector3d& getVisualMeshScale() const;
//   const Eigen::Isometry3d& getVisualMeshOrigin() const;
// }
// // =============================================================================