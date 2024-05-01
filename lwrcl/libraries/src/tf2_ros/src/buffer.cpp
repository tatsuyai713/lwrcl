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

/** \author Wim Meeussen */


#include "tf2_ros/buffer.h"

#include <exception>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

namespace tf2_ros
{

Buffer::Buffer(
  std::shared_ptr<lwrcl::Clock> clock, tf2::Duration cache_time)
: BufferCore(cache_time), clock_(clock)
{
  if (nullptr == clock_) {
    throw std::invalid_argument("clock must be a valid instance");
  }
}

inline
tf2::Duration
from_rclcpp(const lwrcl::Duration & rclcpp_duration)
{
  return tf2::Duration(std::chrono::nanoseconds(rclcpp_duration.nanoseconds()));
}

inline
lwrcl::Duration
to_rclcpp(const tf2::Duration & duration)
{
  return lwrcl::Duration(std::chrono::nanoseconds(duration).count());
}

geometry_msgs::msg::TransformStamped
Buffer::lookupTransform(
  const std::string & target_frame, const std::string & source_frame,
  const tf2::TimePoint & lookup_time, const tf2::Duration timeout) const
{
  // Pass error string to suppress console spam
  std::string error;
  canTransform(target_frame, source_frame, lookup_time, timeout, &error);
  return lookupTransform(target_frame, source_frame, lookup_time);
}

geometry_msgs::msg::TransformStamped
Buffer::lookupTransform(
  const std::string & target_frame, const tf2::TimePoint & target_time,
  const std::string & source_frame, const tf2::TimePoint & source_time,
  const std::string & fixed_frame, const tf2::Duration timeout) const
{
  // Pass error string to suppress console spam
  std::string error;
  canTransform(target_frame, target_time, source_frame, source_time, fixed_frame, timeout, &error);
  return lookupTransform(target_frame, target_time, source_frame, source_time, fixed_frame);
}

void conditionally_append_timeout_info(
  std::string * errstr, const lwrcl::Time & start_time,
  const lwrcl::Time & current_time,
  const lwrcl::Duration & timeout)
{
  if (errstr) {
    std::stringstream ss;
    ss << ". canTransform returned after " <<
      tf2::durationToSec(from_rclcpp(current_time - start_time)) <<
      " timeout was " << tf2::durationToSec(from_rclcpp(timeout)) << ".";
    (*errstr) += ss.str();
  }
}

bool
Buffer::canTransform(
  const std::string & target_frame, const std::string & source_frame,
  const tf2::TimePoint & time, const tf2::Duration timeout, std::string * errstr) const
{
  if (!checkAndErrorDedicatedThreadPresent(errstr)) {
    return false;
  }

  lwrcl::Duration rclcpp_timeout(to_rclcpp(timeout));

  // poll for transform if timeout is set
  lwrcl::Time start_time = clock_->now();
  while (clock_->now() < start_time + rclcpp_timeout &&
    !canTransform(
      target_frame, source_frame, time,
      tf2::Duration(std::chrono::nanoseconds::zero()), errstr) &&
    (clock_->now() + lwrcl::Duration(3, 0) >= start_time))  // don't wait bag loop detected
  {
    // TODO(sloretz) sleep using clock_->sleep_for when implemented
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  bool retval = canTransform(target_frame, source_frame, time, errstr);
  lwrcl::Time current_time = clock_->now();
  conditionally_append_timeout_info(errstr, start_time, current_time, rclcpp_timeout);
  return retval;
}

bool
Buffer::canTransform(
  const std::string & target_frame, const tf2::TimePoint & target_time,
  const std::string & source_frame, const tf2::TimePoint & source_time,
  const std::string & fixed_frame, const tf2::Duration timeout, std::string * errstr) const
{
  if (!checkAndErrorDedicatedThreadPresent(errstr)) {
    return false;
  }

  lwrcl::Duration rclcpp_timeout(to_rclcpp(timeout));

  // poll for transform if timeout is set
  lwrcl::Time start_time = clock_->now();
  while (clock_->now() < start_time + rclcpp_timeout &&
    !canTransform(
      target_frame, target_time, source_frame, source_time, fixed_frame,
      tf2::Duration(std::chrono::nanoseconds::zero()), errstr) &&
    (clock_->now() + lwrcl::Duration(3, 0) >= start_time))  // don't wait bag loop detected
  {
    // TODO(sloretz) sleep using clock_->sleep_for when implemented
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  bool retval = canTransform(
    target_frame, target_time,
    source_frame, source_time, fixed_frame, errstr);
  lwrcl::Time current_time = clock_->now();
  conditionally_append_timeout_info(errstr, start_time, current_time, rclcpp_timeout);
  return retval;
}

bool Buffer::checkAndErrorDedicatedThreadPresent(std::string * error_str) const
{
  if (isUsingDedicatedThread()) {
    return true;
  }

  if (error_str) {
    *error_str = tf2_ros::threading_error;
  }

  printf("%s\n", tf2_ros::threading_error);
  return false;
}

}  // namespace tf2_ros
