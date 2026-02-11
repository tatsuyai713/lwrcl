/*
 * Copyright (c) 2008, Willow Garage, Inc.
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

/** \author Tully Foote */

#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <utility>

#include "tf2_ros/transform_listener.h"

namespace tf2_ros
{

  TransformListener::TransformListener(tf2::BufferCore &buffer, lwrcl::Node::SharedPtr node, bool spin_thread, int32_t domain_id)
      : buffer_(buffer), node_(node), spin_thread_(spin_thread), domain_id_(domain_id)
  {
    init();
  }

  TransformListener::~TransformListener()
  {
    if (spin_thread_)
    {
      executor_->cancel();
      dedicated_listener_thread_->join();
    }
  }

  void TransformListener::subscription_callback(
      tf2_msgs::msg::TFMessage::SharedPtr message,
      bool is_static)
  {

    if (message == nullptr)
    {
      std::cerr << "Error: Received null message in callback." << std::endl;
      return;
    }

    // TODO(tfoote) find a way to get the authority
    std::string authority = "Authority undetectable";
    for (size_t i = 0u; i < message->transforms().size(); i++)
    {
      try
      {
        buffer_.setTransform(message->transforms()[i], authority, is_static);
      }
      catch (const tf2::TransformException &ex)
      {
        // /\todo Use error reporting
        std::string temp = ex.what();
        printf("Failure to set received transform from %s to %s with error: %s\n",
               message->transforms()[i].child_frame_id().c_str(),
               message->transforms()[i].header().frame_id().c_str(), temp.c_str());
      }
    }
  }

} // namespace tf2_ros
