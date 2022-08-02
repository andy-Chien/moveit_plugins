/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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

#include <geometric_shapes/body_operations.h>
#include <ros/console.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include "octomap_updater/point_containment_filter/shape_mask.h"

point_containment_filter::ShapeMask::ShapeMask()
{
}

point_containment_filter::ShapeMask::~ShapeMask()
{
}

bool point_containment_filter::ShapeMask::addShape(const shapes::ShapeConstPtr& shape, std::vector<gpu_voxels::Vector3f>& contain_points, 
                                                    float voxel_size, float scale, float padding)
{
  boost::mutex::scoped_lock _(shapes_lock_);
  const shapes::Mesh* mesh = static_cast<const shapes::Mesh*>(shape.get());
  Eigen::Vector3d scale_indx(1, 1, 1);
  Eigen::Vector3d padding_indx;
  if (mesh->vertex_count > 1)
  {
    double mx = std::numeric_limits<double>::max();
    Eigen::Vector3d min(mx, mx, mx);
    Eigen::Vector3d max(-mx, -mx, -mx);
    unsigned int cnt = mesh->vertex_count * 3;
    for (unsigned int i = 0; i < cnt; i += 3)
    {
      Eigen::Vector3d v(mesh->vertices[i + 0], mesh->vertices[i + 1], mesh->vertices[i + 2]);
      min = min.cwiseMin(v);
      max = max.cwiseMax(v);
    }
    scale_indx = 1.732 * (max - min).normalized(); //1.732 = sqrt(3)
  }
  padding_indx = scale_indx;

  float scale_x, scale_y, scale_z, padding_x, padding_y, padding_z;
  scale_x = (scale > 1) ? 1 + (scale - 1) / scale_indx[0] : scale;
  scale_y = (scale > 1) ? 1 + (scale - 1) / scale_indx[1] : scale;
  scale_z = (scale > 1) ? 1 + (scale - 1) / scale_indx[2] : scale;
  padding_x = (padding > 0) ? padding / padding_indx[0] : padding;
  padding_y = (padding > 0) ? padding / padding_indx[1] : padding;
  padding_z = (padding > 0) ? padding / padding_indx[2] : padding;
  shapes::Shape* shape_for_sample = shape->clone();
  static_cast<shapes::Mesh*>(shape_for_sample)->scaleAndPadd(scale_x, scale_y, scale_z, padding_x, padding_y, padding_z);

  SeeShape ss;
  ss.body = bodies::createBodyFromShape(shape_for_sample);
  if (ss.body)
  {
    ss.volume = ss.body->computeVolume();
    bodies::AABB bounding_box;
    ss.body->computeBoundingBox(bounding_box);
    for(float x = bounding_box.min()[0]; x < bounding_box.max()[0] + voxel_size; x += voxel_size / 2)
    {
      for(float y = bounding_box.min()[1]; y < bounding_box.max()[1] + voxel_size; y += voxel_size / 2)
      {
        for(float z = bounding_box.min()[2]; z < bounding_box.max()[2] + voxel_size; z += voxel_size / 2)
        {
          if (ss.body->containsPoint(Eigen::Vector3d(x, y, z)))
          {
            contain_points.push_back(gpu_voxels::Vector3f(x, y, z));
          }
        }
      }
    }
  }
  else
    return false;
  return true;
}