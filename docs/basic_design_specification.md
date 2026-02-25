# LWRCL Basic Design Specification

## 1. Purpose

This document defines the basic design of **LWRCL (LightWeight Rclcpp Compatible Library)**.  
LWRCL provides a lightweight, ROS 2 `rclcpp`-compatible API for publish/subscribe and service communication without requiring a full ROS 2 runtime installation.

## 2. Scope

This specification covers:

- Core `lwrcl` library design and responsibilities
- Supported communication backends
- Functional and non-functional requirements
- Build, installation, and runtime assumptions
- Verification and known limitations

Out of scope:

- ROS 2 action/lifecycle/component framework internals
- AUTOSAR AP implementation internals outside this repository
- Application-specific logic in downstream projects

## 3. System Context

LWRCL sits between user applications and transport middleware.

- Application API: `lwrcl::*` and `rclcpp::*` compatibility aliases
- Transport backends: `fastdds`, `cyclonedds`, `vsomeip`, `adaptive-autosar`
- Data model: ROS-compatible IDL-generated message/service types

## 4. Supported Platforms and Backends

- Platforms: Linux (Ubuntu/Debian), QNX 8.0
- Backends:
  - Fast DDS
  - CycloneDDS
  - vsomeip (with standalone CDR serialization library)
  - Adaptive AUTOSAR (`ara::com` interface with generated proxy/skeleton)

Backend is selected at build time through `DDS_BACKEND`.

## 5. Functional Requirements

### FR-01 Node Lifecycle

- The system shall provide `init`, `ok`, `shutdown`, `spin`, and `spin_some`.
- A node shall own callbacks, publishers, subscriptions, timers, clients, and services.
- Node shutdown shall stop callbacks and release transport resources.

### FR-02 Topic Communication

- The system shall provide typed publisher/subscription APIs.
- Topic names shall support namespace resolution with ROS-like prefixing.
- Subscription callbacks shall be executed asynchronously under a shared callback mutex.

### FR-03 Service Communication

- The system shall provide typed service server and client APIs.
- Client requests shall support asynchronous future-based response handling.

### FR-04 Execution Model

- Single-threaded and multi-threaded executors shall be provided.
- Executors shall manage one or more nodes and dispatch callbacks safely.

### FR-05 Timer and Time

- The system shall provide wall/system time clocks, durations, rates, and timers.
- Timer callbacks shall be routed through the same callback scheduling mechanism.

### FR-06 Parameters

- The system shall support parameter declaration, get/set, and YAML parameter-file loading.
- ROS-style `--ros-args --param-file <path>` input shall be supported.

### FR-07 QoS

- The system shall provide ROS-style QoS builders and presets.
- History, reliability, durability, liveliness, deadline, lifespan, and lease duration shall be configurable at API level.

### FR-08 Loaned Message API

- The system shall expose loaned message APIs for publish/subscribe paths.
- Behavior may be backend-dependent at runtime.

### FR-09 WaitSet

- The system shall provide polling-based message wait/take APIs in addition to callback-based APIs.

### FR-10 Logging

- The system shall provide logger utilities and ROS-compatible logging macros.

## 6. Non-Functional Requirements

### NFR-01 Portability

- Build scripts and CMake configuration shall support Linux and QNX.
- Backend-specific differences shall be isolated under backend-specific directories.

### NFR-02 Performance

- Callback transport shall use lightweight in-process queues.
- Polling loops shall use bounded sleeps and bounded buffering.
- Loaned messages should avoid copies when backend/runtime allows.

### NFR-03 Reliability

- Shutdown and signal handling shall support graceful termination.
- Runtime errors shall be surfaced as exceptions where appropriate.

### NFR-04 Maintainability

- API compatibility with `rclcpp` naming shall reduce migration cost.
- Core abstractions shall be shared across backends with backend-specific adapter code.

### NFR-05 Testability

- Unit/integration tests shall validate node, pub/sub, service, QoS, timers, waitset, serialization, and executor behavior.
- CI workflows shall validate build and smoke execution for major backend combinations.

## 7. Constraints and Dependencies

- C++14 baseline (vsomeip backend requires C++17 in backend-specific build)
- CMake 3.16.3+
- `yaml-cpp` for parameter and config parsing
- Backend-specific dependencies:
  - Fast DDS stack (`fastrtps`, `fastcdr`, etc.)
  - CycloneDDS stack (optional iceoryx SHM linkage)
  - vsomeip + standalone CDR
  - Adaptive AUTOSAR AP runtime (`AdaptiveAutosarAP`) and generated headers

## 8. Build and Delivery Units

Primary build units:

- `libraries/` (third-party/support libraries)
- `data_types/` (ROS-compatible message/service types)
- `lwrcl/` (core library and tf2/tf2_ros)
- `apps/` (sample and advanced applications)
- `test/` (backend-driven test suite)

Typical build order:

1. `build_libraries.sh`
2. `build_data_types.sh`
3. `build_lwrcl.sh`
4. `build_apps.sh`

## 9. Verification Strategy

- Backend-specific CI workflows validate compilation and smoke tests.
- Adaptive AUTOSAR CI additionally validates:
  - Manifest/mapping/proxy-skeleton generation
  - Runtime transport switching (`dds`, `iceoryx`, `vsomeip`)
  - Loaned-message runtime checks

## 10. Known Limitations

The following are intentionally not supported in the current design:

- ROS 2 lifecycle nodes
- ROS 2 actions
- ROS 2 component framework
- Topic statistics

