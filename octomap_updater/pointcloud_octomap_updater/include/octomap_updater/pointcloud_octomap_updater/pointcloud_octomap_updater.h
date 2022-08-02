/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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

/* Author: Jon Binney, Ioan Sucan */

#ifndef MOVEIT_PERCEPTION_POINTCLOUD_OCTOMAP_UPDATER_
#define MOVEIT_PERCEPTION_POINTCLOUD_OCTOMAP_UPDATER_

#define POINTS_PER_MESH 15000
#define VOXEL_SIDE_LENGTH 0.03f

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/message_filter.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/PointCloud2.h>
#include <moveit/occupancy_map_monitor/occupancy_map_updater.h>
#include <gpu_voxels/GpuVoxels.h>
#include <gpu_voxels/helpers/PointCloud.h>
#include <gpu_voxels/helpers/MetaPointCloud.h>
#include <gpu_voxels/helpers/GeometryGeneration.h>
#include <gpu_voxels/robot/urdf_robot/urdf_robot.h>
#include <gpu_voxels/logging/logging_gpu_voxels.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <icl_core_config/Config.h>
#include "octomap_updater/point_containment_filter/shape_mask.h"
#include <queue>
#include <set>
#include <mutex>
#include <memory>

using boost::dynamic_pointer_cast;
using boost::shared_ptr;
using gpu_voxels::voxelmap::ProbVoxelMap;
using gpu_voxels::voxelmap::DistanceVoxelMap;
using gpu_voxels::voxellist::CountingVoxelList;
using gpu_voxels::voxellist::BitVectorVoxelList;

namespace occupancy_map_monitor
{
class PointCloudOctomapUpdaterFast : public OccupancyMapUpdater
{
public:
  PointCloudOctomapUpdaterFast();
  ~PointCloudOctomapUpdaterFast() override;

  bool setParams(XmlRpc::XmlRpcValue& params) override;

  bool initialize() override;
  void start() override;
  void stop() override;
  ShapeHandle excludeShape(const shapes::ShapeConstPtr& shape) override;
  void forgetShape(ShapeHandle handle) override;

protected:
  virtual void updateMask(const sensor_msgs::PointCloud2& cloud, const Eigen::Vector3d& sensor_origin,
                          std::vector<int>& mask);
  void updateShapeMask();
private:
  uint16_t addCloudToMPC(const std::vector<Vector3f> &cloud);
  ShapeHandle checkShapeExist(const shapes::ShapeConstPtr& shape);
  bool getShapeTransform(ShapeHandle h, Eigen::Isometry3d& transform) const;
  bool updatePointCloud(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg, tf2::Stamped<tf2::Transform> &map_h_sensor);
  void cloudMsgCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);
  void samplePointFromMesh(const shapes::Shape* shape, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr target_cloud, int sample_num);
  void removeShape(ShapeHandle handle);
  void gvlInitialize();
  void stopHelper();
  void emptyOctomap();

  ros::NodeHandle root_nh_;
  ros::NodeHandle private_nh_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  ros::Time last_update_time_;

  /* params */
  std::string point_cloud_topic_;
  std::string robot_state_topic_;
  std::string urdf_path_;
  bool accept_mesh_;
  double scale_;
  double padding_;
  double attached_offset_;
  double attached_scale_;
  double max_range_;
  unsigned int point_subsample_;
  double max_update_rate_;
  std::string filtered_cloud_topic_;
  ros::Publisher filtered_cloud_publisher_;
  ros::Subscriber robot_state_subscriber;

  message_filters::Subscriber<sensor_msgs::PointCloud2>* point_cloud_subscriber_;
  tf2_ros::MessageFilter<sensor_msgs::PointCloud2>* point_cloud_filter_;

  /* used to store all cells in the map which a given ray passes through during raycasting.
     we cache this here because it dynamically pre-allocates a lot of memory in its contsructor */
  octomap::KeyRay key_ray_;

  std::unique_ptr<point_containment_filter::ShapeMask> shape_mask_;
  std::vector<int> mask_;

  shared_ptr<GpuVoxels> gvl;
  Vector3ui map_dimensions;
  float voxel_side_length;
  PointCloud my_point_cloud;
  robot::JointValueMap myRobotJointValues;

  Matrix4f init_transform;
  Matrix4f inv_init_transform;

  shared_ptr<CountingVoxelList> pointCloudVoxelList;
  shared_ptr<BitVectorVoxelList> maskVoxelList;

  tf2::Stamped<tf2::Transform> map_h_sensor;
  std::map<ShapeHandle, shapes::Shape*> contain_shape_;
  std::map<ShapeHandle, Eigen::Isometry3d> tmp_shapes_transform_;
  std::set<ShapeHandle> forget_list_;
  std::queue<ShapeHandle> empty_handle;

  gpu_voxels::MetaPointCloud *shape_mask_mpc;
  gpu_voxels::PointCloud *received_cloud;
  std::mutex mpc_mutex, data_mutex;
};
}  // namespace occupancy_map_monitor

#endif
