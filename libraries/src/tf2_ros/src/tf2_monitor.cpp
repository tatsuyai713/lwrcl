/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * Copyright (c) 2015, Open Source Robotics Foundation, Inc.
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

/** \author Wim Meeussen */

#include <algorithm>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "rcl_like_wrapper.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

class TFMonitor
{
public:
  std::string framea_, frameb_;
  bool using_specific_chain_;

  std::shared_ptr<rcl_like_wrapper::RCLWNode> node_;
  intptr_t subscriber_tf_, subscriber_tf_message_;
  std::vector<std::string> chain_;
  std::map<std::string, std::string> frame_authority_map;
  std::map<std::string, std::vector<double>> delay_map;
  std::map<std::string, std::vector<double>> authority_map;
  std::map<std::string, std::vector<double>> authority_frequency_map;

  std::shared_ptr<rcl_like_wrapper::Clock> clock_;
  tf2_ros::Buffer buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_;

  tf2_msgs::msg::TFMessage message_;
  std::mutex map_mutex_;

  void callback(void *in_message)
{
    if (in_message == nullptr)
    {
        std::cerr << "Error: Received null message in callback." << std::endl;
        return;
    }

    tf2_msgs::msg::TFMessage* message_ptr = static_cast<tf2_msgs::msg::TFMessage *>(in_message);

    const tf2_msgs::msg::TFMessage & message = *(message_ptr);
    // TODO(tfoote): recover authority info
    std::string authority = "<no authority available>";

    double average_offset = 0;
    std::unique_lock<std::mutex> my_lock(map_mutex_);
    for (size_t i = 0; i < message.transforms.size(); i++) {
      frame_authority_map[message.transforms[i].child_frame_id] = authority;

      double offset = clock_->now().seconds() - tf2_ros::timeToSec(
        message.transforms[i].header.stamp);
      average_offset += offset;

      std::map<std::string, std::vector<double>>::iterator it = delay_map.find(
        message.transforms[i].child_frame_id);
      if (it == delay_map.end()) {
        delay_map[message.transforms[i].child_frame_id] = std::vector<double>(1, offset);
      } else {
        it->second.push_back(offset);
        if (it->second.size() > 1000) {
          it->second.erase(it->second.begin());
        }
      }
    }

    average_offset /= std::max(static_cast<size_t>(1), message.transforms.size());

    // create the authority log
    std::map<std::string, std::vector<double>>::iterator it2 = authority_map.find(authority);
    if (it2 == authority_map.end()) {
      authority_map[authority] = std::vector<double>(1, average_offset);
    } else {
      it2->second.push_back(average_offset);
      if (it2->second.size() > 1000) {
        it2->second.erase(it2->second.begin());
      }
    }

    // create the authority frequency log
    std::map<std::string, std::vector<double>>::iterator it3 = authority_frequency_map.find(
      authority);
    if (it3 == authority_frequency_map.end()) {
      authority_frequency_map[authority] = std::vector<double>(1, clock_->now().seconds());
    } else {
      it3->second.push_back(clock_->now().seconds());
      if (it3->second.size() > 1000) {
        it3->second.erase(it3->second.begin());
      }
    }
  }

  TFMonitor(
    std::shared_ptr<rcl_like_wrapper::RCLWNode> node, bool using_specific_chain,
    std::string framea = "", std::string frameb = "")
  : framea_(framea),
    frameb_(frameb),
    using_specific_chain_(using_specific_chain),
    node_(node)
  {
    clock_ = std::make_shared<rcl_like_wrapper::Clock>(0),
    tf_ = std::make_shared<tf2_ros::TransformListener>(buffer_);

    buffer_ = tf2_ros::Buffer(clock_, tf2::Duration(tf2::BUFFER_CORE_DEFAULT_CACHE_TIME), node)

    if (using_specific_chain_) {
      std::string warning_msg;
      while (!buffer_.canTransform(
          framea_, frameb_, tf2::TimePoint(), &warning_msg))
      {
        printf(
          "Waiting for transform %s ->  %s: %s\n", framea_.c_str(), frameb_.c_str(),
          warning_msg.c_str());
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
      }

      try {
        buffer_._chainAsVector(
          frameb_, tf2::TimePointZero, framea_, tf2::TimePointZero, frameb_,
          chain_);
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(node->get_logger(), "Transform Exception %s", ex.what());
        return;
      }
    }

    eprosima::fastdds::dds::TopicQos topic_qos = eprosima::fastdds::dds::TOPIC_QOS_DEFAULT;
    subscriber_tf_ = node_->create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf", topic_qos,
      std::bind(&TFMonitor::callback, this, std::placeholders::_1));
    subscriber_tf_message_ = node_->create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf_static", topic_qos,
      std::bind(&TFMonitor::callback, this, std::placeholders::_1));
  }

  std::string outputFrameInfo(
    const std::map<std::string, std::vector<double>>::iterator & it,
    const std::string & frame_authority)
  {
    std::stringstream ss;
    double average_delay = 0;
    double max_delay = 0;
    for (size_t i = 0; i < it->second.size(); i++) {
      average_delay += it->second[i];
      max_delay = std::max(max_delay, it->second[i]);
    }
    average_delay /= it->second.size();
    ss << "Frame: " << it->first << ", published by " << frame_authority << ", Average Delay: " <<
      average_delay << ", Max Delay: " << max_delay << std::endl;
    return ss.str();
  }

  void spin()
  {
    // create tf listener
    double max_diff = 0;
    double avg_diff = 0;
    double lowpass = 0.01;
    unsigned int counter = 0;

    if (using_specific_chain_) {
      std::cout << "Gathering data on " << framea_ << " -> " << frameb_ << " for 10 seconds...\n";
    } else {
      std::cout << "Gathering data on all frames for 10 seconds...\n";
    }


    while (1) {
      counter++;
      if (using_specific_chain_) {
        auto tmp = buffer_.lookupTransform(framea_, frameb_, tf2::TimePointZero);
        double diff = clock_->now().seconds() - tf2_ros::timeToSec(tmp.header.stamp);
        avg_diff = lowpass * diff + (1 - lowpass) * avg_diff;
        if (diff > max_diff) {
          max_diff = diff;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      if (counter > 20) {
        counter = 0;

        if (using_specific_chain_) {
          std::cout << std::endl << std::endl << std::endl << "RESULTS: for " << framea_ <<
            " to " << frameb_ << std::endl;
          std::cout << "Chain is: ";
          for (size_t i = 0; i < chain_.size(); i++) {
            std::cout << chain_[i];
            if (i != chain_.size() - 1) {
              std::cout << " -> ";
            }
          }
          std::cout << std::endl;
          std::cout << "Net delay " << "    avg = " << avg_diff << ": max = " << max_diff <<
            std::endl;
        } else {
          std::cout << std::endl << std::endl << std::endl << "RESULTS: for all Frames" <<
            std::endl;
        }
        std::unique_lock<std::mutex> lock(map_mutex_);
        std::cout << std::endl << "Frames:" << std::endl;
        std::map<std::string, std::vector<double>>::iterator it = delay_map.begin();
        for ( ; it != delay_map.end(); ++it) {
          if (using_specific_chain_) {
            for (size_t i = 0; i < chain_.size(); i++) {
              if (it->first != chain_[i]) {
                continue;
              }

              std::cout << outputFrameInfo(it, frame_authority_map[it->first]);
            }
          } else {
            std::cout << outputFrameInfo(it, frame_authority_map[it->first]);
          }
        }
        std::cerr << std::endl << "All Broadcasters:" << std::endl;
        std::map<std::string, std::vector<double>>::iterator it1 = authority_map.begin();
        std::map<std::string, std::vector<double>>::iterator it2 = authority_frequency_map.begin();
        for ( ; it1 != authority_map.end(); ++it1, ++it2) {
          double average_delay = 0;
          double max_delay = 0;
          for (size_t i = 0; i < it1->second.size(); i++) {
            average_delay += it1->second[i];
            max_delay = std::max(max_delay, it1->second[i]);
          }
          average_delay /= it1->second.size();
          double frequency_out = static_cast<double>(it2->second.size()) /
            std::max(0.00000001, (it2->second.back() - it2->second.front()));
          std::cout << "Node: " << it1->first << " " << frequency_out << " Hz, Average Delay: " <<
            average_delay << " Max Delay: " << max_delay << std::endl;
        }
      }
    }
  }
};


int main(int argc, char ** argv)
{

  // TODO(tfoote): make anonymous
  std::shared_ptr<rcl_like_wrapper::RCLWNode> nh = rcl_like_wrapper::RCLWNode::make_shared(0);

  std::string framea, frameb;
  bool using_specific_chain = true;
  if (args.size() == 3) {
    framea = args[1];
    frameb = args[2];
  } else if (args.size() == 1) {
    using_specific_chain = false;
  } else {
    printf("TF_Monitor: usage: tf2_monitor framea frameb");
    return -1;
  }

  // TODO(tfoote) restore simtime logic
  // //Make sure we don't start before recieving time when in simtime
  // int iterations = 0;
  // while (ros::Time::now() == ros::Time())
  // {
  //   if (++iterations > 10)
  //   {
  //     RCLCPP_INFO(nh->get_logger(), "tf2_monitor waiting for time to be published");
  //     iterations = 0;
  //   }
  //   ros::WallDuration(0.1).sleep();
  // }

  // This lambda is required because `std::thread` cannot infer the correct
  // rcl_like_wrapper::spin, since there are more than one versions of it (overloaded).
  // see: http://stackoverflow.com/a/27389714/671658
  // I (wjwwood) chose to use the lamda rather than the static cast solution.
  auto run_func = [](std::shared_ptr<rcl_like_wrapper::RCLWNode> node) {
      return rcl_like_wrapper::spin(node->get_node_pointer());
    };
  TFMonitor monitor(nh, using_specific_chain, framea, frameb);
  std::thread spinner(run_func, nh);

  monitor.spin();
  spinner.join();
  return 0;
}
