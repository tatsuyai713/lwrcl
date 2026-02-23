# LWRCL API Specification

## 1. Scope

This document specifies the public API exposed by `lwrcl` and the `rclcpp` compatibility layer in this repository.

## 2. Namespaces

- Primary API namespace: `lwrcl`
- Compatibility namespace: `rclcpp` (mostly type aliases and wrapper functions)

## 3. Global Runtime API (`lwrcl`)

### 3.1 Lifecycle

- `void init(int argc, char *argv[])`
  - Initializes runtime and optional parameter-file loading.
- `bool ok()`
  - Returns `true` while global runtime is active.
- `void shutdown()`
  - Signals global stop and deinitializes runtime.
- `void spin(std::shared_ptr<lwrcl::Node> node)`
  - Blocking callback processing loop for one node.
- `void spin_some(std::shared_ptr<lwrcl::Node> node)`
  - Non-blocking callback drain for one node.
- `void sleep_for(const lwrcl::Duration &duration)`
  - Sleeps for duration.

### 3.2 Parameter File Helpers

- `std::string get_params_file_path(int argc, char *argv[])`
- `void load_parameters(const std::string &file_path)`

### 3.3 Future Utilities

- `template <typename Duration> FutureReturnCode spin_until_future_complete(...)`
- Return codes:
  - `SUCCESS`
  - `INTERRUPTED`
  - `TIMEOUT`

## 4. Core Types

## 4.1 Node

Construction:

- `Node::make_shared(...)` overloads for:
  - domain id + optional name/namespace/options
  - existing domain participant + optional name/namespace

Getters:

- `get_participant()`
- `get_name()`
- `get_namespace()`
- `get_fully_qualified_name()`
- `get_node_options()`
- `get_logger()`
- `get_clock()`

Communication factories:

- `create_publisher<T>(topic, depth|QoS)`
- `create_subscription<T>(topic, depth|QoS, callback)`
- `create_service<T>(service_name, callback)`
- `create_client<T>(service_name)`
- `create_timer(period, callback)`
- `create_wall_timer(period, callback)`

Parameter operations:

- `declare_parameter(name, default_value)` for bool/int/double/string/arrays
- `get_parameter(name)` and typed `get_parameter(name, out)`
- `set_parameters(vector<Parameter>)`
- `set_parameters(vector<shared_ptr<ParameterBase>>)`

Node runtime:

- `shutdown()`
- `stop_spin()`

## 4.2 Publisher

Type: `template <typename T> class Publisher`

Key methods:

- `publish(const std::shared_ptr<T>&)`
- `publish(const T&)`
- `publish(T&)`
- `publish(LoanedMessage<T>&&)`
- `borrow_loaned_message()`
- `can_loan_messages()`
- `get_subscriber_count()`
- `get_subscription_count()` (compat alias)
- `get_topic_name()`

## 4.3 LoanedMessage

Type: `template <typename T> class LoanedMessage`

Key methods:

- `is_valid()`
- `is_loaned()`
- `get()`
- pointer/indirection operators (`->`, `*`)

## 4.4 Subscription

Type: `template <typename T> class Subscription`

Key methods:

- `get_publisher_count()`
- `stop()`
- `take(T &out_msg, MessageInfo &info)`
- `has_message()`
- `has_data()` (compat alias)
- `take_loaned_message(LoanedSubscriptionMessage<T> &out_loaned)`
- `can_loan_messages()`
- `get_message_count()`
- `get_topic_name()`

## 4.5 LoanedSubscriptionMessage

Type: `template <typename T> class LoanedSubscriptionMessage`

Key methods:

- `is_valid()`
- `get()`
- `get_sample_info()`
- `release()`
- pointer/indirection operators (`->`, `*`)

## 4.6 WaitSet

Type: `class WaitSet`

Key methods:

- `add_subscription(std::shared_ptr<Subscription<T>>)`
- `wait()`
- `wait(duration)`
- `wait_for(std::chrono::nanoseconds)`
- `get_subscriptions()`

Related types:

- `WaitResult`
- `WaitResultKind` (`Ready`, `Timeout`, `Error`)
- `MessageInfo`

## 4.7 Service

Type: `template <typename T> class Service`

Key behavior:

- Construct with request callback
- Receives request and publishes response
- `stop()` terminates request subscription

## 4.8 Client

Type: `template <typename T> class Client`

Key methods:

- `SharedFuture async_send_request(SharedRequest request)`
- `wait_for_service(timeout)`
- `stop()`

## 4.9 Executors

`lwrcl::executors::SingleThreadedExecutor`:

- `add_node`, `remove_node`, `spin`, `spin_some`, `cancel`, `clear`

`lwrcl::executors::MultiThreadedExecutor`:

- `add_node`, `remove_node`, `spin`, `spin_some`, `cancel`, `clear`
- `get_number_of_threads()`

## 4.10 Timer and Time API

`TimerBase`:

- `start()`, `stop()`, `cancel()`, `reset()`
- `is_canceled()`, `is_ready()`, `get_period()`

Time primitives:

- `Time`, `Duration`, `Clock`
- `Rate`, `WallRate`

## 4.11 Parameter Types

`ParameterBase`:

- `get_name()`
- `as_string()`

`Parameter` constructors support:

- bool, int, double, string, char*
- bool/int/double/string arrays
- byte array

Typed conversion API:

- `as_bool()`, `as_int()`, `as_double()`, `as_string()`
- `as_bool_array()`, `as_integer_array()`, `as_double_array()`, `as_string_array()`, `as_byte_array()`

## 4.12 QoS API

Core classes and types:

- `QoSInitialization`
- `QoS`
- `RMWQoSProfile`, `RMWDuration`
- `KeepLast`, `KeepAll`

Preset QoS classes:

- `SensorDataQoS`
- `SystemDefaultsQoS`
- `ServicesQoS`
- `ParametersQoS`
- `ParameterEventsQoS`
- `BestEffortQoS`
- `ReliableQoS`

`QoS` fluent methods include:

- `keep_last`, `keep_all`
- `reliability`, `reliable`, `best_effort`
- `durability`, `transient_local`, `durability_volatile`
- `liveliness`, `deadline`, `lifespan`, `liveliness_lease_duration`
- `to_rmw_qos_profile()`

## 4.13 Logging

- `Logger` class with `log(level, format, ...)`
- `log(level, format, ...)` global helper
- Macros:
  - `LWRCL_DEBUG/INFO/WARN/ERROR`
  - `RCLCPP_DEBUG/INFO/WARN/ERROR`
  - simplified `_THROTTLE` and `_ONCE` macro variants in compatibility headers

## 5. Compatibility Layer (`rclcpp`)

`rclcpp/rclcpp.hpp` provides:

- Type aliases to `lwrcl` classes (Node, QoS, Publisher, Subscription, Client, Service, Executor, etc.)
- Wrapper functions:
  - `rclcpp::init`, `ok`, `spin`, `spin_some`, `shutdown`, `sleep_for`, `spin_until_future_complete`

## 6. Error and Exception Behavior

Representative runtime errors:

- Null node pointers passed to spin functions
- Missing parameter access
- Invalid loaned message publish attempts
- Backend adapter initialization failure

Errors are primarily reported via `std::runtime_error`.

## 7. Backend-Specific Notes

- API shape is intentionally similar across backends.
- Runtime behavior can differ by backend:
  - loaning implementation path
  - QoS enforcement capabilities
  - transport-level discovery and subscription counting behavior

