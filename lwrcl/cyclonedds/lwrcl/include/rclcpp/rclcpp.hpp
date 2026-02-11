#ifndef RCLCPP_HPP_
#define RCLCPP_HPP_
#include "lwrcl.hpp"

namespace rclcpp {
    using DomainParticipant = lwrcl::DomainParticipant;
    using Clock = lwrcl::Clock;
    using Node = lwrcl::Node;
    namespace executors {
        using SingleThreadedExecutor = lwrcl::executors::SingleThreadedExecutor;
        using MultiThreadedExecutor = lwrcl::executors::MultiThreadedExecutor;
    }
    using Duration = lwrcl::Duration;
    using Time = lwrcl::Time;
    using Rate = lwrcl::Rate;
    using WallRate = lwrcl::WallRate;
    using Parameter = lwrcl::Parameter;
    template <typename T>
    using Service = ::lwrcl::Service<T>;
    template <typename T>
    using Client = ::lwrcl::Client<T>;
    using FutureBase = lwrcl::FutureBase;
    template <typename T>
    using TypedFuture = ::lwrcl::TypedFuture<T>;
    using FutureReturnCode = lwrcl::FutureReturnCode;
    template <typename MessageType>
    using Subscription = ::lwrcl::Subscription<MessageType>;
    template <typename MessageType>
    using Publisher = ::lwrcl::Publisher<MessageType>;
    
    using TimerBase = lwrcl::TimerBase;

    using WaitSet = lwrcl::WaitSet;
    using WaitResult = lwrcl::WaitResult;
    using WaitResultKind = lwrcl::WaitResultKind;
    using MessageInfo = lwrcl::MessageInfo;
    

    inline bool ok() {
        return lwrcl::ok();
    }
    inline void spin(std::shared_ptr<Node> node) {
        return lwrcl::spin(node);
    }
    inline void spin_some(std::shared_ptr<Node> node) {
        return lwrcl::spin_some(node);
    }
    inline void shutdown() {
        return lwrcl::shutdown();
    }
    inline void sleep_for(const Duration &duration) {
        return lwrcl::sleep_for(duration);
    }
    inline void init(int argc, char *argv[]) {
        return lwrcl::init(argc, argv);
    }
    template <typename Duration>
    inline FutureReturnCode spin_until_future_complete(std::shared_ptr<Node> node,
      std::shared_ptr<FutureBase> future, const Duration & timeout){
        return lwrcl::spin_until_future_complete(node, future, timeout);
    }
}

#define RCLCPP_DEBUG(logger, ...) (logger).log(lwrcl::DEBUG, __VA_ARGS__)
#define RCLCPP_INFO(logger, ...) (logger).log(lwrcl::INFO, __VA_ARGS__)
#define RCLCPP_WARN(logger, ...) (logger).log(lwrcl::WARN, __VA_ARGS__)
#define RCLCPP_ERROR(logger, ...) (logger).log(lwrcl::ERROR, __VA_ARGS__)

#endif // RCLCPP_HPP_
