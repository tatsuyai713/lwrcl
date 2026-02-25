# LWRCL Detailed Design Specification

## 1. Overview

This document describes implementation-level design for the `lwrcl` core library and backend adaptations.

## 2. Module Decomposition

### 2.1 Core Runtime Modules

- `lwrcl/include/lwrcl.hpp`
  - Public API aggregation
  - `Node`, `Service`, `Client`, executor classes, parameter API
- `lwrcl/include/timer.hpp`
  - Timer scheduling and callback dispatch integration
- `lwrcl/include/clock_time_duration.hpp`
  - Time, duration, and clock abstractions
- `lwrcl/include/qos.hpp`
  - QoS model and preset profiles
- `lwrcl/include/publisher.hpp`, `subscription.hpp`
  - Typed pub/sub APIs and loaned-message support
- `lwrcl/src/lwrcl.cpp`
  - Runtime implementation, signal handling, parameter loading, executor logic

### 2.2 Backend-Specific Implementations

- `lwrcl/fastdds/*`
- `lwrcl/cyclonedds/*`
- `lwrcl/vsomeip/*`
- `lwrcl/adaptive-autosar/*`

Each backend provides equivalent API shape with transport-specific internals.

## 3. Runtime Execution Model

### 3.1 Callback Dispatch

- Callbacks are invoked directly under a shared `callback_mutex_` per node.
- Producers:
  - Subscription waitset workers
  - Timer worker threads
  - Other asynchronous adapters
- Consumers:
  - `Node::spin()` (blocking loop)
  - `Node::spin_some()` (non-blocking drain)

### 3.2 Node Event Loop

- `Node::spin()`:
  - Loop while node is open and global stop is false
  - Consume callback objects and invoke `invoke()`
- `Node::spin_some()`:
  - Drain currently queued callbacks without blocking

### 3.3 Executors

- `SingleThreadedExecutor`:
  - Iterates managed nodes and calls `spin_some()`
- `MultiThreadedExecutor`:
  - Starts one thread per node and calls `spin()`
- `cancel()` and `clear()` provide stop and cleanup semantics.

## 4. Core Class Design

## 4.1 `Node`

Responsibilities:

- Maintain logical node identity (name/namespace/options)
- Hold domain participant handle
- Own communication entities and timer objects
- Resolve topic names (`rt/` namespace strategy)
- Own parameter state for declared parameters

Important members:

- `participant_`, `callback_mutex_`, `clock_`
- entity lists: publishers, subscriptions, timers, services, clients
- parameter map: `parameters_`

Lifecycle:

1. Construct with domain id or existing participant
2. Create entities via templated factory methods
3. Execute callbacks through `spin`/`spin_some`
4. `shutdown()` stops and clears all entity lists

## 4.2 Publisher and Loaned Publisher Message

- Publisher owns transport-side writer endpoint object.
- Standard publish overloads:
  - `publish(const std::shared_ptr<T>&)`
  - `publish(const T&)`
  - `publish(T&)`
- Loaned publish path:
  - `borrow_loaned_message()`
  - `publish(LoanedMessage<T>&&)`

Adaptive AUTOSAR details:

- Uses generated `TopicEventSkeleton<T>`.
- Offers service/event during construction.
- Stops offer on destruction.
- Falls back to heap allocation if transport loan allocation is unavailable.

## 4.3 Subscription and Loaned Subscription Message

- Subscription owns transport-side reader endpoint object.
- Internal worker polls new samples and:
  - queues callback objects into callback channel
  - stores pollable copies/loaned references in bounded deques
- APIs:
  - `take(T&, MessageInfo&)`
  - `has_message()`
  - `take_loaned_message(LoanedSubscriptionMessage<T>&)`

Buffer policy:

- `MAX_POLLABLE_BUFFER_SIZE` bounds pollable and loaned buffer depth.

## 4.4 Service and Client

Service design:

- Request topic: `"rp/<service>_Request"`
- Response topic: `"rp/<service>_Response"`
- Service callback creates response object and publishes response.

Client design:

- Sends request via request publisher
- Receives response via response subscription
- Maps responses to queued promises
- `async_send_request()` returns shared future
- `wait_for_service()` polls availability by subscriber count

## 4.5 Timer

- Each `TimerBase` owns a worker thread.
- Worker schedules periodic wakeups using `sleep_until`.
- On timeout, invokes the timer callback under `callback_mutex_`.
- Supports `cancel()`, `reset()`, `is_canceled()`, `is_ready()`.

## 4.6 Parameter Subsystem

- Global parameter store: `node_parameters`
- Per-node store: `parameters_`
- `declare_parameter()`:
  - uses loaded YAML value when present
  - otherwise uses provided default
- `set_parameters()`:
  - updates declared entries
  - rejects missing node/parameter combinations
- `get_parameter()` throws if undeclared/missing

## 4.7 Time, Duration, Rate

- `Time` and `Duration` are nanosecond-based value objects.
- `Clock` supports `SYSTEM_TIME` and `STEADY_TIME`.
- `Rate` and `WallRate` implement drift-aware periodic sleep.

## 4.8 QoS

- API-level QoS settings follow ROS-like builder style.
- QoS can be translated to `RMWQoSProfile`.
- Presets: `SensorDataQoS`, `ServicesQoS`, `ParametersQoS`, etc.

Note:

- Backend enforcement varies; Adaptive AUTOSAR code notes QoS as fixed by runtime layer.

## 4.9 WaitSet

- `WaitSet` stores subscription adapters through `SubscriptionHolder`.
- `wait()` / `wait_for()` polls `has_message()` across subscriptions.
- `WaitResultKind` indicates `Ready`, `Timeout`, or `Error`.

## 5. Initialization and Shutdown Design

`init(argc, argv)`:

1. Reset global stop flag
2. Initialize backend runtime (Adaptive AUTOSAR path includes `ara::core::Initialize()`)
3. Install signal handlers (`SIGINT`, `SIGTERM`)
4. Parse optional parameter file argument and load YAML

`shutdown()`:

1. Set global stop flag
2. Deinitialize runtime where required (Adaptive AUTOSAR path calls `ara::core::Deinitialize()`)

## 6. Concurrency and Thread Safety

- Subscription waitset and callback delivery are protected by `callback_mutex_`.
- Executor node lists are guarded by executor-local mutexes.
- Timer state uses atomics.
- Potentially blocking APIs are isolated (`spin`, wait loops, timer workers).

## 7. Error Handling Strategy

- Constructor failures and invalid runtime states throw `std::runtime_error`.
- Invalid pointer usage in core APIs raises explicit runtime errors.
- Callback execution catches exceptions to prevent worker thread termination.

## 8. Adaptive AUTOSAR-Specific Design Points

- Build-time generated artifacts:
  - topic mapping YAML
  - binding manifests (`dds`, `iceoryx`, `vsomeip`, `auto`)
  - ARXML manifest
  - generated proxy/skeleton header
- Runtime transport switching via environment variables:
  - `ARA_COM_EVENT_BINDING` preferred
  - `ARA_COM_BINDING_MANIFEST` fallback
- Generated proxy/skeleton is mandatory for app build in adaptive-autosar mode.

## 9. Test Coverage Mapping

The `test/` suite validates:

- Node, pub/sub, service/client
- Executors (`single`, `multi`)
- WaitSet and `spin_some`
- Timer and timer control
- Parameter and QoS APIs
- Time/duration/rate
- Serialization
- Zero-copy (Fast DDS and CycloneDDS paths)

