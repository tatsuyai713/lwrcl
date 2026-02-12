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

#ifndef RCLCPP_KEEPLAST_DEFINED
#define RCLCPP_KEEPLAST_DEFINED
    // Helper functions
    inline QoSInitialization KeepLast(size_t depth) {
        return lwrcl::KeepLast(depth);
    }
    inline QoSInitialization KeepAll() {
        return lwrcl::KeepAll();
    }
#endif
} // namespace rclcpp

#endif // RCLCPP_QOS_HPP_