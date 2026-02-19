/*
 * Copyright (c) 2019, Open Source Robotics Foundation, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef TF2_ROS__STATIC_TRANSFORM_BROADCASTER_NODE_HPP_
#define TF2_ROS__STATIC_TRANSFORM_BROADCASTER_NODE_HPP_

#include <memory>

#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster_visibility_control.h"

#include "lwrcl.hpp"

namespace tf2_ros
{
  struct TransformData
  {
    double translation_x;
    double translation_y;
    double translation_z;
    double rotation_x;
    double rotation_y;
    double rotation_z;
    double rotation_w;
    std::string frame_id;
    std::string child_frame_id;

    TransformData(double tx, double ty, double tz, double rx, double ry, double rz, double rw, std::string fid, std::string cfid)
        : translation_x(tx), translation_y(ty), translation_z(tz),
          rotation_x(rx), rotation_y(ry), rotation_z(rz), rotation_w(rw),
          frame_id(fid), child_frame_id(cfid) {}
  };
  class StaticTransformBroadcasterNode final : public lwrcl::Node
  {
  public:
    STATIC_TRANSFORM_BROADCASTER_PUBLIC
    explicit StaticTransformBroadcasterNode(int domain_id, TransformData transformData);

    STATIC_TRANSFORM_BROADCASTER_PUBLIC
    ~StaticTransformBroadcasterNode() override = default;
    bool init(const std::string &config_file_path)
    {
      (void)config_file_path;
      return true;
    }

  private:
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
  };

} // namespace tf2_ros

#endif // TF2_ROS__STATIC_TRANSFORM_BROADCASTER_NODE_HPP_
