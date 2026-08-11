#ifndef RCLCPP_RATE_HPP_
#define RCLCPP_RATE_HPP_

#include "lwrcl.hpp"

namespace rclcpp {
    using Clock = lwrcl::Clock;
    using Duration = lwrcl::Duration;
    using Time = lwrcl::Time;
    using Rate = lwrcl::Rate;
    using WallRate = lwrcl::WallRate;
} // namespace rclcpp

#endif // RCLCPP_RATE_HPP_
