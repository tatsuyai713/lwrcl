#ifndef RCLCPP_SERVICE_HPP_
#define RCLCPP_SERVICE_HPP_

#include "lwrcl.hpp"

namespace rclcpp {
    template <typename ServiceT>
    using Service = lwrcl::Service<ServiceT>;
} // namespace rclcpp

#endif // RCLCPP_SERVICE_HPP_
