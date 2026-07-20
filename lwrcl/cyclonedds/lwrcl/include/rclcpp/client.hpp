#ifndef RCLCPP_CLIENT_HPP_
#define RCLCPP_CLIENT_HPP_

#include "lwrcl.hpp"

namespace rclcpp {
    template <typename ServiceT>
    using Client = lwrcl::Client<ServiceT>;
} // namespace rclcpp

#endif // RCLCPP_CLIENT_HPP_
