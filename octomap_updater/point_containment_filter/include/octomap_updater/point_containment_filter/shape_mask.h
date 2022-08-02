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

#ifndef MOVEIT_POINT_CONTAINMENT_FILTER_SELF_MASK_
#define MOVEIT_POINT_CONTAINMENT_FILTER_SELF_MASK_

#include <sensor_msgs/PointCloud2.h>
#include <geometric_shapes/bodies.h>
#include <gpu_voxels/helpers/cuda_vectors.h>
#include <boost/function.hpp>
#include <vector>
#include <set>
#include <map>

#include <boost/thread/mutex.hpp>

namespace point_containment_filter
{
typedef unsigned int ShapeHandle;

/** \brief Computing a mask for a pointcloud that states which points are inside the robot */
class ShapeMask
{
public:
  /** \brief The possible values of a mask computed for a point */
  enum
  {
    INSIDE = 0,
    OUTSIDE = 1,
    CLIP = 2
  };

  // typedef boost::function<bool(ShapeHandle, Eigen::Isometry3d&)> TransformCallback;

  /** \brief Construct the filter */
  ShapeMask();

  /** \brief Destructor to clean up */
  virtual ~ShapeMask();

  bool addShape(const shapes::ShapeConstPtr& shape, std::vector<gpu_voxels::Vector3f>& contain_points, 
                float voxel_size, float scale, float padding);

protected:
  struct SeeShape
  {
    SeeShape()
    {
      body = NULL;
    }

    bodies::Body* body;
    ShapeHandle handle;
    double volume;
  };

  /** \brief Protects, bodies_ and bspheres_. All public methods acquire this mutex for their whole duration. */
  mutable boost::mutex shapes_lock_;
};
}  // namespace point_containment_filter
#endif
