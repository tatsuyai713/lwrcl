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

#ifndef TF2_ROS__TRANSFORM_LISTENER_H_
#define TF2_ROS__TRANSFORM_LISTENER_H_

#include <functional>
#include <memory>
#include <thread>
#include <utility>

#include "lwrcl.hpp"
#include "tf2/buffer_core.h"
#include "tf2/time.h"
#include "tf2_ros/visibility_control.h"

#include "tf2_msgs/msg/tf_message.hpp"

namespace tf2_ros
{
  class TFListenerNode : public lwrcl::Node
  {
  public:
    TFListenerNode(uint16_t domain_id) : Node(domain_id) {}
  };

  /** \brief This class provides an easy way to request and receive coordinate frame transform information.
   */
  class TransformListener
  {
  public:
    /**@brief Constructor for transform listener */
    TF2_ROS_PUBLIC
    TransformListener(tf2::BufferCore &buffer, lwrcl::Node::SharedPtr node, bool spin_thread = true, int32_t domain_id = 0);
    TF2_ROS_PUBLIC
    virtual ~TransformListener();

  private:
    void init()
    {
      
      auto cb = std::bind(
          &TransformListener::subscription_callback, this, std::placeholders::_1, false);
      auto static_cb = std::bind(
          &TransformListener::subscription_callback, this, std::placeholders::_1, true);

      if (spin_thread_)
      {
        tf_listener_node_ = std::make_shared<TFListenerNode>(domain_id_);
        executor_ = std::make_shared<lwrcl::executors::SingleThreadedExecutor>();
        message_subscription_tf_ = tf_listener_node_->create_subscription<tf2_msgs::msg::TFMessage>("tf", 10, std::move(cb));
        message_subscription_tf_static_ = tf_listener_node_->create_subscription<tf2_msgs::msg::TFMessage>("tf_static", 10, std::move(static_cb));
        executor_->add_node(tf_listener_node_);
        dedicated_listener_thread_ = std::make_unique<std::thread>([&]()
                                                                   { executor_->spin(); });
      }
      else
      {
        message_subscription_tf_ = node_->create_subscription<tf2_msgs::msg::TFMessage>("tf", 10, std::move(cb));
        message_subscription_tf_static_ = node_->create_subscription<tf2_msgs::msg::TFMessage>("tf_static", 10, std::move(static_cb));
        
      }
    }
    /// Callback function for ros message subscriptoin
    TF2_ROS_PUBLIC
    void subscription_callback(tf2_msgs::msg::TFMessage::SharedPtr msg, bool is_static);

    tf2::BufferCore &buffer_;
    lwrcl::Node::SharedPtr node_;
    bool spin_thread_{false};
    int32_t domain_id_{0};
    std::unique_ptr<std::thread> dedicated_listener_thread_{nullptr};
    std::shared_ptr<TFListenerNode> tf_listener_node_;
    std::shared_ptr<lwrcl::executors::SingleThreadedExecutor> executor_;
    std::shared_ptr<lwrcl::Subscription<tf2_msgs::msg::TFMessage>> message_subscription_tf_{0};
    std::shared_ptr<lwrcl::Subscription<tf2_msgs::msg::TFMessage>> message_subscription_tf_static_{0};
  };
} // namespace tf2_ros

#endif // TF2_ROS__TRANSFORM_LISTENER_H_
