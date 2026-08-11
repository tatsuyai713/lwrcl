#ifndef RCLCPP_SUBSCRIPTION_HPP_
#define RCLCPP_SUBSCRIPTION_HPP_

#include "lwrcl.hpp"

namespace rclcpp {
    template <typename MessageT>
    using Subscription = lwrcl::Subscription<MessageT>;
    template <typename MessageT>
    using LoanedSubscriptionMessage = lwrcl::LoanedSubscriptionMessage<MessageT>;
    using MessageInfo = lwrcl::MessageInfo;
} // namespace rclcpp

#endif // RCLCPP_SUBSCRIPTION_HPP_
