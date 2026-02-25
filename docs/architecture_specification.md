# LWRCL Architecture Specification

## 1. Architectural Goals

LWRCL architecture is designed to:

- Provide a lightweight `rclcpp`-compatible programming model
- Isolate transport specifics behind backend implementations
- Support multiple middleware stacks with a consistent application-facing API
- Enable automotive-oriented deployment, including Adaptive AUTOSAR integration

## 2. Architectural Style

LWRCL follows a layered architecture with backend-specific adapters.

Logical layers:

1. Application Layer
   - User nodes and sample apps under `apps/`
2. API Layer
   - `lwrcl` public API and `rclcpp` compatibility headers
3. Core Runtime Layer
   - Node lifecycle, executors, callback dispatch, timers, parameters, QoS
4. Backend Adapter Layer
   - Fast DDS / CycloneDDS / vsomeip / Adaptive AUTOSAR implementations
5. Transport Runtime Layer
   - Actual middleware/runtime processes and libraries

## 3. Static Architecture

Repository architecture map:

- `lwrcl/`
  - backend-specific implementations (`fastdds`, `cyclonedds`, `vsomeip`, `adaptive-autosar`)
  - `tf2` and `tf2_ros` integration per backend
- `data_types/`
  - ROS-compatible message/service definitions and generated types
- `apps/`
  - examples and advanced scenario apps
- `test/`
  - backend-oriented validation suite
- `scripts/`
  - dependency installation, build tooling, and Adaptive AUTOSAR codegen helpers

## 4. Build-Time Architecture

Backend selection:

- Controlled by `DDS_BACKEND` in CMake and build scripts.
- Supported values:
  - `fastdds`
  - `cyclonedds`
  - `vsomeip`
  - `adaptive-autosar`

Build stages:

1. Support libraries
2. Data types
3. Core `lwrcl` library
4. Applications

Install prefixes vary per backend (for example `/opt/fast-dds-libs`, `/opt/cyclonedds-libs`, `/opt/vsomeip-libs`, `/opt/autosar-ap-libs` on Linux).

## 5. Runtime Architecture

## 5.1 Callback and Execution Topology

- Subscriptions and timers invoke callbacks under a shared mutex per node.
- Node spin loops consume and execute callbacks.
- Executors orchestrate one or more nodes:
  - single-thread polling dispatch
  - multi-thread node-parallel dispatch

## 5.2 Communication Topology

Topic communication:

- Typed publisher/subscription API
- Namespace-aware topic resolution
- Support for callback-driven and waitset-driven receive models

Service communication:

- Request/response topic pair pattern (`_Request`, `_Response`)
- Async request with future-based completion

## 6. Backend Architecture

## 6.1 Fast DDS Backend

- Transport via Fast DDS/RTPS stack
- Links against Fast DDS libraries
- Supports standard pub/sub, services, QoS, and zero-copy APIs

## 6.2 CycloneDDS Backend

- Transport via CycloneDDS (`ddsc`, `ddscxx`)
- Optional iceoryx shared-memory linkage depending on environment

## 6.3 vsomeip Backend

- Transport via `vsomeip3`
- Serialization via standalone CDR (`lwrcl_cdr`)
- Does not require a DDS runtime for message transport

## 6.4 Adaptive AUTOSAR Backend

- API integration via `AdaptiveAutosarAP::ara_com`/`ara_core`
- Uses generated proxy/skeleton classes for topic endpoints
- CycloneDDS/vsomeip/iceoryx can be selected through Adaptive AUTOSAR binding profiles

## 7. Adaptive AUTOSAR Code Generation Architecture

At app build time (`build_apps.sh adaptive-autosar`):

1. Scan app sources for communication declarations
2. Generate topic mapping YAML
3. Generate binding manifests (auto/dds/iceoryx/vsomeip profiles)
4. Generate proxy/skeleton header (`lwrcl_autosar_proxy_skeleton.hpp`)
5. Optionally generate ARXML manifest
6. Install generated artifacts under `share/lwrcl/autosar`

This keeps the runtime API stable while allowing transport binding to be selected externally.

## 8. Runtime Binding Selection (Adaptive AUTOSAR)

Primary runtime selector:

- `ARA_COM_EVENT_BINDING` (`dds`, `iceoryx`, `vsomeip`, `auto`)

Fallback selector:

- `ARA_COM_BINDING_MANIFEST`

Supporting environment variables include mapping path overrides and vsomeip configuration path.

## 9. Deployment View

Typical deployment units:

- Shared library: `liblwrcl.so`
- Generated message/service type libraries
- Sample or product application binaries
- Optional runtime processes (routing manager, `iox-roudi`, etc. depending on backend/profile)

In Adaptive AUTOSAR profiles, deployment also includes generated manifests and proxy/skeleton artifacts.

## 10. Concurrency and Fault Isolation

- Node-local callback mutexes isolate callback execution per node.
- Timers and subscription polling use dedicated worker threads.
- Executor-level mutexes protect node collections.
- Callback invocation catches exceptions to avoid unintentional worker termination.

## 11. Quality Attributes and Trade-offs

Primary quality attributes:

- API compatibility and developer productivity
- Transport flexibility and deployability
- Lightweight runtime composition

Trade-offs:

- Some backend features are normalized to a common subset
- QoS behavior may differ by transport/runtime capabilities
- Polling-based wait loops prefer simplicity over event-driven complexity

## 12. Verification Architecture

Validation is split across:

- Unit/integration tests in `test/`
- Backend CI workflows for build and smoke execution
- Adaptive AUTOSAR CI checks for generated artifact integrity and runtime transport switching

