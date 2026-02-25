#ifndef RCLCPP_HPP_
#define RCLCPP_HPP_
#include "lwrcl.hpp"

namespace rclcpp {
    using DomainParticipant = lwrcl::DomainParticipant;
    using Clock = lwrcl::Clock;
    using Node = lwrcl::Node;
    using NodeOptions = lwrcl::Node::NodeOptions;
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
    template <typename MessageType>
    using LoanedMessage = ::lwrcl::LoanedMessage<MessageType>;
    template <typename MessageType>
    using LoanedSubscriptionMessage = ::lwrcl::LoanedSubscriptionMessage<MessageType>;

    using TimerBase = lwrcl::TimerBase;

    using WaitSet = lwrcl::WaitSet;
    using WaitResult = lwrcl::WaitResult;
    using WaitResultKind = lwrcl::WaitResultKind;
    using MessageInfo = lwrcl::MessageInfo;

    // QoS Profiles
    using QoS = lwrcl::QoS;
    using SensorDataQoS = lwrcl::SensorDataQoS;
    using SystemDefaultsQoS = lwrcl::SystemDefaultsQoS;
    using ServicesQoS = lwrcl::ServicesQoS;
    using ParametersQoS = lwrcl::ParametersQoS;
    using ParameterEventsQoS = lwrcl::ParameterEventsQoS;
    using BestEffortQoS = lwrcl::BestEffortQoS;
    using ReliableQoS = lwrcl::ReliableQoS;
    using QoSInitialization = lwrcl::QoSInitialization;


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
    template <typename ResponseT, typename Duration>
    inline FutureReturnCode spin_until_future_complete(std::shared_ptr<Node> node,
      std::shared_future<ResponseT> & future, const Duration & timeout){
        return lwrcl::spin_until_future_complete(node, future, timeout);
    }
}

// Include QoS helper functions from qos.hpp
#include "rclcpp/qos.hpp"

// Basic logger macros
#define RCLCPP_DEBUG(logger, ...) (logger).log(lwrcl::DEBUG, __VA_ARGS__)
#define RCLCPP_INFO(logger, ...) (logger).log(lwrcl::INFO, __VA_ARGS__)
#define RCLCPP_WARN(logger, ...) (logger).log(lwrcl::WARN, __VA_ARGS__)
#define RCLCPP_ERROR(logger, ...) (logger).log(lwrcl::ERROR, __VA_ARGS__)

// Stream logger macros (accepts std::ostream-style expressions)
#define RCLCPP_DEBUG_STREAM(logger, stream_arg) do { \
    std::ostringstream _lwrcl_ss_; _lwrcl_ss_ << stream_arg; \
    (logger).log(lwrcl::DEBUG, "%s", _lwrcl_ss_.str().c_str()); } while(0)
#define RCLCPP_INFO_STREAM(logger, stream_arg) do { \
    std::ostringstream _lwrcl_ss_; _lwrcl_ss_ << stream_arg; \
    (logger).log(lwrcl::INFO, "%s", _lwrcl_ss_.str().c_str()); } while(0)
#define RCLCPP_WARN_STREAM(logger, stream_arg) do { \
    std::ostringstream _lwrcl_ss_; _lwrcl_ss_ << stream_arg; \
    (logger).log(lwrcl::WARN, "%s", _lwrcl_ss_.str().c_str()); } while(0)
#define RCLCPP_ERROR_STREAM(logger, stream_arg) do { \
    std::ostringstream _lwrcl_ss_; _lwrcl_ss_ << stream_arg; \
    (logger).log(lwrcl::ERROR, "%s", _lwrcl_ss_.str().c_str()); } while(0)

// Throttled logger macros (fires at most once per duration_ms milliseconds per call site)
#define RCLCPP_DEBUG_THROTTLE(logger, clock, duration_ms, ...) do { \
    static std::chrono::steady_clock::time_point _lwrcl_last_ = \
        std::chrono::steady_clock::time_point::min(); \
    auto _lwrcl_now_ = std::chrono::steady_clock::now(); \
    if (std::chrono::duration_cast<std::chrono::milliseconds>( \
            _lwrcl_now_ - _lwrcl_last_).count() >= static_cast<long long>(duration_ms)) { \
        _lwrcl_last_ = _lwrcl_now_; (logger).log(lwrcl::DEBUG, __VA_ARGS__); } } while(0)
#define RCLCPP_INFO_THROTTLE(logger, clock, duration_ms, ...) do { \
    static std::chrono::steady_clock::time_point _lwrcl_last_ = \
        std::chrono::steady_clock::time_point::min(); \
    auto _lwrcl_now_ = std::chrono::steady_clock::now(); \
    if (std::chrono::duration_cast<std::chrono::milliseconds>( \
            _lwrcl_now_ - _lwrcl_last_).count() >= static_cast<long long>(duration_ms)) { \
        _lwrcl_last_ = _lwrcl_now_; (logger).log(lwrcl::INFO, __VA_ARGS__); } } while(0)
#define RCLCPP_WARN_THROTTLE(logger, clock, duration_ms, ...) do { \
    static std::chrono::steady_clock::time_point _lwrcl_last_ = \
        std::chrono::steady_clock::time_point::min(); \
    auto _lwrcl_now_ = std::chrono::steady_clock::now(); \
    if (std::chrono::duration_cast<std::chrono::milliseconds>( \
            _lwrcl_now_ - _lwrcl_last_).count() >= static_cast<long long>(duration_ms)) { \
        _lwrcl_last_ = _lwrcl_now_; (logger).log(lwrcl::WARN, __VA_ARGS__); } } while(0)
#define RCLCPP_ERROR_THROTTLE(logger, clock, duration_ms, ...) do { \
    static std::chrono::steady_clock::time_point _lwrcl_last_ = \
        std::chrono::steady_clock::time_point::min(); \
    auto _lwrcl_now_ = std::chrono::steady_clock::now(); \
    if (std::chrono::duration_cast<std::chrono::milliseconds>( \
            _lwrcl_now_ - _lwrcl_last_).count() >= static_cast<long long>(duration_ms)) { \
        _lwrcl_last_ = _lwrcl_now_; (logger).log(lwrcl::ERROR, __VA_ARGS__); } } while(0)

// Stream + throttle macros
#define RCLCPP_DEBUG_STREAM_THROTTLE(logger, clock, duration_ms, stream_arg) do { \
    static std::chrono::steady_clock::time_point _lwrcl_last_ = \
        std::chrono::steady_clock::time_point::min(); \
    auto _lwrcl_now_ = std::chrono::steady_clock::now(); \
    if (std::chrono::duration_cast<std::chrono::milliseconds>( \
            _lwrcl_now_ - _lwrcl_last_).count() >= static_cast<long long>(duration_ms)) { \
        _lwrcl_last_ = _lwrcl_now_; \
        std::ostringstream _lwrcl_ss_; _lwrcl_ss_ << stream_arg; \
        (logger).log(lwrcl::DEBUG, "%s", _lwrcl_ss_.str().c_str()); } } while(0)
#define RCLCPP_INFO_STREAM_THROTTLE(logger, clock, duration_ms, stream_arg) do { \
    static std::chrono::steady_clock::time_point _lwrcl_last_ = \
        std::chrono::steady_clock::time_point::min(); \
    auto _lwrcl_now_ = std::chrono::steady_clock::now(); \
    if (std::chrono::duration_cast<std::chrono::milliseconds>( \
            _lwrcl_now_ - _lwrcl_last_).count() >= static_cast<long long>(duration_ms)) { \
        _lwrcl_last_ = _lwrcl_now_; \
        std::ostringstream _lwrcl_ss_; _lwrcl_ss_ << stream_arg; \
        (logger).log(lwrcl::INFO, "%s", _lwrcl_ss_.str().c_str()); } } while(0)
#define RCLCPP_WARN_STREAM_THROTTLE(logger, clock, duration_ms, stream_arg) do { \
    static std::chrono::steady_clock::time_point _lwrcl_last_ = \
        std::chrono::steady_clock::time_point::min(); \
    auto _lwrcl_now_ = std::chrono::steady_clock::now(); \
    if (std::chrono::duration_cast<std::chrono::milliseconds>( \
            _lwrcl_now_ - _lwrcl_last_).count() >= static_cast<long long>(duration_ms)) { \
        _lwrcl_last_ = _lwrcl_now_; \
        std::ostringstream _lwrcl_ss_; _lwrcl_ss_ << stream_arg; \
        (logger).log(lwrcl::WARN, "%s", _lwrcl_ss_.str().c_str()); } } while(0)
#define RCLCPP_ERROR_STREAM_THROTTLE(logger, clock, duration_ms, stream_arg) do { \
    static std::chrono::steady_clock::time_point _lwrcl_last_ = \
        std::chrono::steady_clock::time_point::min(); \
    auto _lwrcl_now_ = std::chrono::steady_clock::now(); \
    if (std::chrono::duration_cast<std::chrono::milliseconds>( \
            _lwrcl_now_ - _lwrcl_last_).count() >= static_cast<long long>(duration_ms)) { \
        _lwrcl_last_ = _lwrcl_now_; \
        std::ostringstream _lwrcl_ss_; _lwrcl_ss_ << stream_arg; \
        (logger).log(lwrcl::ERROR, "%s", _lwrcl_ss_.str().c_str()); } } while(0)

// Once logger macros (logs exactly once per call site)
#define RCLCPP_DEBUG_ONCE(logger, ...) do { \
    static bool _lwrcl_once_ = false; \
    if (!_lwrcl_once_) { _lwrcl_once_ = true; (logger).log(lwrcl::DEBUG, __VA_ARGS__); } \
} while(0)
#define RCLCPP_INFO_ONCE(logger, ...) do { \
    static bool _lwrcl_once_ = false; \
    if (!_lwrcl_once_) { _lwrcl_once_ = true; (logger).log(lwrcl::INFO, __VA_ARGS__); } \
} while(0)
#define RCLCPP_WARN_ONCE(logger, ...) do { \
    static bool _lwrcl_once_ = false; \
    if (!_lwrcl_once_) { _lwrcl_once_ = true; (logger).log(lwrcl::WARN, __VA_ARGS__); } \
} while(0)
#define RCLCPP_ERROR_ONCE(logger, ...) do { \
    static bool _lwrcl_once_ = false; \
    if (!_lwrcl_once_) { _lwrcl_once_ = true; (logger).log(lwrcl::ERROR, __VA_ARGS__); } \
} while(0)

// Stream + once macros
#define RCLCPP_DEBUG_STREAM_ONCE(logger, stream_arg) do { \
    static bool _lwrcl_once_ = false; \
    if (!_lwrcl_once_) { _lwrcl_once_ = true; \
        std::ostringstream _lwrcl_ss_; _lwrcl_ss_ << stream_arg; \
        (logger).log(lwrcl::DEBUG, "%s", _lwrcl_ss_.str().c_str()); } } while(0)
#define RCLCPP_INFO_STREAM_ONCE(logger, stream_arg) do { \
    static bool _lwrcl_once_ = false; \
    if (!_lwrcl_once_) { _lwrcl_once_ = true; \
        std::ostringstream _lwrcl_ss_; _lwrcl_ss_ << stream_arg; \
        (logger).log(lwrcl::INFO, "%s", _lwrcl_ss_.str().c_str()); } } while(0)
#define RCLCPP_WARN_STREAM_ONCE(logger, stream_arg) do { \
    static bool _lwrcl_once_ = false; \
    if (!_lwrcl_once_) { _lwrcl_once_ = true; \
        std::ostringstream _lwrcl_ss_; _lwrcl_ss_ << stream_arg; \
        (logger).log(lwrcl::WARN, "%s", _lwrcl_ss_.str().c_str()); } } while(0)
#define RCLCPP_ERROR_STREAM_ONCE(logger, stream_arg) do { \
    static bool _lwrcl_once_ = false; \
    if (!_lwrcl_once_) { _lwrcl_once_ = true; \
        std::ostringstream _lwrcl_ss_; _lwrcl_ss_ << stream_arg; \
        (logger).log(lwrcl::ERROR, "%s", _lwrcl_ss_.str().c_str()); } } while(0)

#endif // RCLCPP_HPP_
