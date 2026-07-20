#ifndef RCLCPP_PUBLISHER_HPP_
#define RCLCPP_PUBLISHER_HPP_

#include "lwrcl.hpp"

namespace rclcpp {
    template <typename MessageT>
    using Publisher = lwrcl::Publisher<MessageT>;
    template <typename MessageT>
    using LoanedMessage = lwrcl::LoanedMessage<MessageT>;
} // namespace rclcpp

#endif // RCLCPP_PUBLISHER_HPP_
