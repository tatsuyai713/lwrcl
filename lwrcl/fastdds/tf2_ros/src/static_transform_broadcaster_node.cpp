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

#include <memory>
#include <random>
#include <stdexcept>
#include <string>

#include "tf2_ros/static_transform_broadcaster_node.hpp"
#include "lwrcl.hpp"

namespace tf2_ros
{
StaticTransformBroadcasterNode::StaticTransformBroadcasterNode(int domain_id, TransformData transformData)
: lwrcl::Node(domain_id)
// TODO(clalancette): Anonymize the node name like it is in ROS1.
{
  geometry_msgs::msg::TransformStamped tf_msg;

  lwrcl::Clock clock;
  lwrcl::Time now;
  now = clock.now();
  tf_msg.header().stamp().sec() = (int32_t)now.seconds();
  tf_msg.header().stamp().nanosec() = (uint32_t)now.nanoseconds();
  tf_msg.transform().translation().x() = transformData.translation_x;
  tf_msg.transform().translation().y() = transformData.translation_y;
  tf_msg.transform().translation().z() = transformData.translation_z;
  tf_msg.transform().rotation().x() = transformData.rotation_x;
  tf_msg.transform().rotation().y() = transformData.rotation_y;
  tf_msg.transform().rotation().z() = transformData.rotation_z;
  tf_msg.transform().rotation().w() = transformData.rotation_w;
  tf_msg.header().frame_id() = transformData.frame_id;
  tf_msg.child_frame_id() = transformData.child_frame_id;

  // check frame_id != child_frame_id
  if (tf_msg.header().frame_id() == tf_msg.child_frame_id()) {
    printf("cannot publish static transform from '%s' to '%s', exiting\n",
      tf_msg.header().frame_id().c_str(), tf_msg.child_frame_id().c_str());
    throw std::runtime_error("child_frame_id cannot equal frame_id");
  }

  broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(shared_from_this());

  // send transform
  broadcaster_->sendTransform(tf_msg);
}
}  // namespace tf2_ros

