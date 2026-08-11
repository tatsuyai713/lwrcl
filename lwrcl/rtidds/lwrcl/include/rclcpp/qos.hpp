#ifndef RCLCPP_QOS_HPP_
#define RCLCPP_QOS_HPP_
#include "lwrcl.hpp"

namespace rclcpp {
    using QoS = lwrcl::QoS;
    using SensorDataQoS = lwrcl::SensorDataQoS;
    using SystemDefaultsQoS = lwrcl::SystemDefaultsQoS;
    using ServicesQoS = lwrcl::ServicesQoS;
    using ParametersQoS = lwrcl::ParametersQoS;
    using ParameterEventsQoS = lwrcl::ParameterEventsQoS;
    using BestEffortQoS = lwrcl::BestEffortQoS;
    using ReliableQoS = lwrcl::ReliableQoS;
    using QoSInitialization = lwrcl::QoSInitialization;

    using KeepLast = lwrcl::KeepLast;
    using KeepAll = lwrcl::KeepAll;
} // namespace rclcpp

#endif // RCLCPP_QOS_HPP_