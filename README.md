[![FastDDS CI](https://github.com/tatsuyai713/lwrcl/actions/workflows/ci-fastdds.yml/badge.svg)](https://github.com/tatsuyai713/lwrcl/actions/workflows/ci-fastdds.yml)
[![CycloneDDS CI](https://github.com/tatsuyai713/lwrcl/actions/workflows/ci-cyclonedds.yml/badge.svg)](https://github.com/tatsuyai713/lwrcl/actions/workflows/ci-cyclonedds.yml)
[![vsomeip CI](https://github.com/tatsuyai713/lwrcl/actions/workflows/ci-vsomeip.yml/badge.svg)](https://github.com/tatsuyai713/lwrcl/actions/workflows/ci-vsomeip.yml)
[![Adaptive AUTOSAR CI](https://github.com/tatsuyai713/lwrcl/actions/workflows/ci-adaptive-autosar.yml/badge.svg)](https://github.com/tatsuyai713/lwrcl/actions/workflows/ci-adaptive-autosar.yml)

**[日本語版 (Japanese)](README_JA.md)**

# LWRCL (LightWeight Rclcpp Compatible Library)

**A lightweight DDS communication library with an rclcpp-compatible API**

lwrcl provides an API compatible with ROS 2's rclcpp. It supports **CycloneDDS**, **FastDDS**, **vsomeip (SOME/IP)**, and **Adaptive AUTOSAR (ara::com)** as communication backends, enabling direct topic and service communication with ROS 2 nodes without a full ROS 2 installation.

---

## Features

- **rclcpp-compatible API** — Uses the `rclcpp` namespace, allowing existing ROS 2 code to be ported with minimal changes. Logging macros such as `RCLCPP_INFO` and `RCLCPP_WARN` are also available.
- **Interoperability with ROS 2** — Communicates directly with ROS 2 nodes via topics and services on the same DDS domain.
- **DDS backend selection** — Choose between CycloneDDS, FastDDS, vsomeip, or Adaptive AUTOSAR at build time. Multiple backends can be installed simultaneously and switched as needed.
- **Minimal dependencies** — Depends only on a DDS library and yaml-cpp, resulting in fast builds.
- **Multi-platform** — Supports Linux (Ubuntu/Debian) and QNX 8.0.
- **Includes tf2 / tf2_ros** — Bundled coordinate transformation support.

---

## Supported DDS Implementations

| DDS Implementation | Description |
|-------------------|-------------|
| **CycloneDDS** | Eclipse Foundation open-source implementation. Lightweight. |
| **FastDDS** | By eProsima. Rich QoS options. |
| **vsomeip** | COVESA SOME/IP implementation. Automotive-grade transport without DDS runtime dependency. |
| **Adaptive AUTOSAR** | `ara::com`-based backend (Adaptive-AUTOSAR integration) running on CycloneDDS. |

---

## Feature Comparison

| Feature | lwrcl | rclcpp |
|---------|:-----:|:------:|
| Node | ✅ | ✅ |
| Publisher / Subscription | ✅ | ✅ |
| Service / Client | ✅ | ✅ |
| Timer (create_timer / create_wall_timer) | ✅ | ✅ |
| Executor (Single / Multi-threaded) | ✅ | ✅ |
| QoS (Reliability, Durability, History) | ✅ | ✅ |
| QoS (Deadline, Lifespan, Liveliness) | ✅ | ✅ |
| QoS Presets (SensorDataQoS, etc.) | ✅ | ✅ |
| Parameter (declare, get, set) | ✅ | ✅ |
| Parameter File (YAML) | ✅ | ✅ |
| Zero Copy (Loaned Messages) | ✅ | ✅ |
| WaitSet | ✅ | ✅ |
| Namespace Support | ✅ | ✅ |
| Time / Duration / Clock / Rate | ✅ | ✅ |
| Logging (DEBUG, INFO, WARN, ERROR) | ✅ | ✅ |
| Serialization / Deserialization | ✅ | ✅ |
| tf2 / tf2_ros | ✅ | ✅ |
| Lifecycle Node | ❌ | ✅ |
| Action | ❌ | ✅ |
| Component | ❌ | ✅ |
| Topic Statistics | ❌ | ✅ |
| Intra-process Communication | ✅ (DDS-dependent) | ✅ |

---

## Dependencies

- CMake 3.16.3 or later
- C++14-compatible compiler
- Communication backend (CycloneDDS, FastDDS, vsomeip, or Adaptive AUTOSAR)
- yaml-cpp (included as a submodule)
- Boost (required for vsomeip backend)

---

## Build Instructions (Linux)

All build scripts follow this pattern:

```
./build_<target>.sh <fastdds|cyclonedds|vsomeip|adaptive-autosar> [install|clean]
```

- 1st argument: Communication backend (`fastdds`, `cyclonedds`, `vsomeip`, or `adaptive-autosar`)
- 2nd argument: `install` to build & install, `clean` to remove build directory

### Prerequisites

- Linux (Ubuntu 22.04 / 24.04 recommended)

> **Note**: If ROS 2 is installed, comment out `source /opt/ros/*/setup.bash` in your `~/.bashrc`. lwrcl does not depend on ROS 2, but having ROS 2 environment variables set may cause build conflicts.

### 1. Clone the Repository

```bash
git clone --recursive <REPOSITORY_URL>
cd lwrcl
```

### 2. Install Communication Backend

Install the backend you want to use. You may install multiple backends without issues.

**CycloneDDS:**

```bash
./scripts/install_cyclonedds.sh
source ~/.bashrc
```

**FastDDS (Ubuntu/Debian):**

```bash
./scripts/install_fast_dds.sh
source ~/.bashrc
```

> For Arch Linux, use `install_fast_dds_archlinux.sh` instead.

**vsomeip (SOME/IP):**

```bash
./scripts/install_vsomeip.sh
```

> vsomeip requires Boost and CycloneDDS (for the `idlc` code generator at build time only). Install CycloneDDS first, then run `install_vsomeip.sh`. The vsomeip backend does **not** depend on a DDS runtime — it uses CDR serialization extracted from cyclonedds-cxx as a standalone static library.

**Adaptive AUTOSAR (ara::com):**

- Install Adaptive AUTOSAR AP runtime to `/opt/autosar_ap` (for example with [`Adaptive-AUTOSAR`](https://github.com/tatsuyai713/Adaptive-AUTOSAR)).
- CycloneDDS is still required as the DDS runtime backend used by `ara::com`.
- Build with `adaptive-autosar` backend (see below).

### 3. Build Support Libraries

```bash
./build_libraries.sh fastdds install
```

### 4. Build ROS Data Types

Builds ROS 2 compatible message types (`std_msgs`, `sensor_msgs`, `geometry_msgs`, etc.).

```bash
./build_data_types.sh fastdds install
```

### 5. Build lwrcl

```bash
./build_lwrcl.sh fastdds install
```

### 6. Build Sample Applications (Optional)

```bash
./build_apps.sh fastdds install
```

Built binaries are placed in `apps/install-fastdds/`.

> **To use CycloneDDS, replace `fastdds` with `cyclonedds` in the commands above.**
> **To use vsomeip, replace `fastdds` with `vsomeip` in the commands above.**
> **For Adaptive AUTOSAR, run the backend-specific steps below.**

### Adaptive AUTOSAR Build Flow

Adaptive AUTOSAR backend uses `ara::com` APIs. It still requires `yaml-cpp`, so run `build_libraries.sh adaptive-autosar install` first.

```bash
./build_libraries.sh adaptive-autosar install
./build_data_types.sh adaptive-autosar install
./build_lwrcl.sh adaptive-autosar install
./build_apps.sh adaptive-autosar install
```

Built sample binaries are placed in `apps/install-adaptive-autosar/`.

Adaptive AUTOSAR backend is now ARXML-mediated:

- App build-time (`build_apps.sh adaptive-autosar`) auto-generates mapping/manifest by scanning app source calls (`create_publisher`, `create_subscription`, `create_service`, `create_client`) via `autosar-generate-comm-manifest` (installed from Adaptive-AUTOSAR into `PATH`).
- App build-time also generates a proxy/skeleton header from mapping via `autosar-generate-proxy-skeleton` (installed from Adaptive-AUTOSAR into `PATH`).
- Generated manifest includes both `msg` topics and `srv` request/response topics.
- Generated artifacts are installed to:
  - `/opt/autosar-ap-libs/share/lwrcl/autosar/lwrcl_autosar_manifest.arxml`
  - `/opt/autosar-ap-libs/share/lwrcl/autosar/lwrcl_autosar_topic_mapping.yaml`
  - `/opt/autosar-ap-libs/share/lwrcl/autosar/lwrcl_autosar_manifest.yaml`
  - `/opt/autosar-ap-libs/share/lwrcl/autosar/lwrcl_autosar_manifest_dds.yaml`
  - `/opt/autosar-ap-libs/share/lwrcl/autosar/lwrcl_autosar_manifest_iceoryx.yaml` (when codegen supports `event_binding: iceoryx`)
  - `/opt/autosar-ap-libs/share/lwrcl/autosar/lwrcl_autosar_manifest_vsomeip.yaml`
- Generated app-local proxy/skeleton header:
  - `apps/build-adaptive-autosar/autosar/generated/lwrcl_autosar_proxy_skeleton.hpp`
- Runtime `adaptive-autosar` Publisher/Subscription use generated `ara::com` Proxy/Skeleton classes, and transport selection is driven by `ARA_COM_BINDING_MANIFEST` profile manifest.

Required Adaptive-AUTOSAR codegen commands:

- `autosar-generate-comm-manifest`
- `autosar-generate-proxy-skeleton`
- Default install location: `/opt/autosar_ap/bin` (added to `PATH` by Adaptive AUTOSAR build scripts)
- Script source location in Adaptive-AUTOSAR project: `tools/ara_com_codegen/`
- If `autosar-generate-comm-manifest --help` does not list `iceoryx` in `--event-binding`, update Adaptive-AUTOSAR codegen tools or override `AUTOSAR_COMM_MANIFEST_GENERATOR`.

Adaptive AUTOSAR mapping-related environment variables:

| Variable | Purpose |
|----------|---------|
| `ARA_COM_TOPIC_MAPPING` | Runtime path override for topic mapping YAML |
| `ARA_COM_REQUIRE_TOPIC_MAPPING=1` | Fail when topic is not found in mapping |
| `ARA_COM_DISABLE_TOPIC_MAPPING=1` | Disable mapping and use direct DDS topic names |
| `ARA_COM_BINDING_MANIFEST` | Runtime binding profile manifest path (`event_binding: dds`, `iceoryx`, or `vsomeip`) |
| `AUTOSAR_APP_SOURCE_ROOT` | App source root to scan for topic/service usage |
| `AUTOSAR_ARXML_GENERATOR` | Build-time override for ARXML generator script path |
| `AUTOSAR_COMM_MANIFEST_GENERATOR` | Build-time override for mapping generator command (default: `autosar-generate-comm-manifest`) |
| `AUTOSAR_PROXY_SKELETON_GENERATOR` | Build-time override for proxy/skeleton generator command (default: `autosar-generate-proxy-skeleton`) |
| `AUTOSAR_EVENT_BINDING` | Build-time default `event_binding` for `lwrcl_autosar_manifest.yaml` (default: `auto`) |
| `AUTOSAR_GENERATE_BINDING_PROFILES=1` | Also generate/install `lwrcl_autosar_manifest_dds.yaml`, `lwrcl_autosar_manifest_iceoryx.yaml`, and `lwrcl_autosar_manifest_vsomeip.yaml` |
| `VSOMEIP_PREFIX` | Build-time vsomeip install prefix override (default: `/opt/vsomeip`) |
| `VSOMEIP_CONFIGURATION` | Runtime vsomeip configuration file path |

Adaptive AUTOSAR runtime transport switch (same app binary, no app code change):

```bash
# CycloneDDS transport profile
# (current Adaptive-AUTOSAR reference runtime requires routing manager process)
unset ARA_COM_EVENT_BINDING
export ARA_COM_BINDING_MANIFEST=/opt/autosar-ap-libs/share/lwrcl/autosar/lwrcl_autosar_manifest_dds.yaml
export VSOMEIP_CONFIGURATION=/opt/autosar_ap/configuration/vsomeip-rpi.json
/opt/autosar_ap/bin/autosar_vsomeip_routing_manager &
apps/install-adaptive-autosar/bin/example_class_sub &
apps/install-adaptive-autosar/bin/example_class_pub

# iceoryx transport profile
unset ARA_COM_EVENT_BINDING
export ARA_COM_BINDING_MANIFEST=/opt/autosar-ap-libs/share/lwrcl/autosar/lwrcl_autosar_manifest_iceoryx.yaml
iox-roudi &
apps/install-adaptive-autosar/bin/example_class_sub &
apps/install-adaptive-autosar/bin/example_class_pub

# SOME/IP transport profile
unset ARA_COM_EVENT_BINDING
export ARA_COM_BINDING_MANIFEST=/opt/autosar-ap-libs/share/lwrcl/autosar/lwrcl_autosar_manifest_vsomeip.yaml
export VSOMEIP_CONFIGURATION=/opt/autosar_ap/configuration/vsomeip-rpi.json
/opt/autosar_ap/bin/autosar_vsomeip_routing_manager &
apps/install-adaptive-autosar/bin/example_class_sub &
apps/install-adaptive-autosar/bin/example_class_pub
```

Switch verification checklist:

- DDS profile: subscriber prints `I heard: 'Hello, world! ...'`, and routing manager log does not show `REGISTER EVENT`.
- iceoryx profile: subscriber prints `I heard: 'Hello, world! ...'` with `iox-roudi` running and no runtime transport errors.
- SOME/IP profile: subscriber prints `I heard: 'Hello, world! ...'`, and routing manager log shows `REGISTER EVENT` / `SUBSCRIBE`.

### Cleaning Build Directories

```bash
./build_lwrcl.sh fastdds clean
```

> Build directories are separated by backend (`build-fastdds` / `build-cyclonedds` / `build-vsomeip` / `build-adaptive-autosar`). Switching backends does not require cleaning.

### Installation Paths

| Backend | DDS Install Path | lwrcl Install Path |
|---------|-----------------|-------------------|
| FastDDS | `/opt/fast-dds` | `/opt/fast-dds-libs` |
| CycloneDDS | `/opt/cyclonedds` | `/opt/cyclonedds-libs` |
| vsomeip | `/opt/vsomeip` | `/opt/vsomeip-libs` |
| Adaptive AUTOSAR | `/opt/autosar_ap` | `/opt/autosar-ap-libs` (includes `yaml-cpp` via `build_libraries.sh adaptive-autosar`) |

---

## CycloneDDS Zero-Copy (iceoryx)

For CycloneDDS backend, lwrcl uses native writer-loan/read-loan APIs when available and falls back safely when loaning is unavailable.

To enable SHM zero-copy transport with iceoryx:

```bash
./scripts/install_iceoryx.sh
./scripts/install_cyclonedds.sh --enable-shm
export LD_LIBRARY_PATH=/opt/iceoryx/lib:/opt/cyclonedds/lib:/opt/cyclonedds-libs/lib:${LD_LIBRARY_PATH}
export CYCLONEDDS_URI=file:///opt/cyclonedds/etc/cyclonedds-lwrcl.xml
iox-roudi
```

`install_iceoryx.sh` applies a container-safe ACL fallback by default (if `/dev/shm` ACL is unsupported).  
If you need strict ACL enforcement, reinstall with:

```bash
./scripts/install_iceoryx.sh --force --strict-acl
```

Container-side setting (preferred if host supports tmpfs ACL):

```bash
docker run ... \
  --tmpfs /dev/shm:rw,nosuid,nodev,noexec,size=4g,mode=1777,acl \
  <image>
```

```yaml
services:
  app:
    tmpfs:
      - /dev/shm:rw,nosuid,nodev,noexec,size=4g,mode=1777,acl
```

If the host kernel/filesystem does not support tmpfs POSIX ACL, container settings alone cannot enable ACL.

Then build and run with `cyclonedds` backend (`example_zero_copy_pub` / `example_zero_copy_sub`).

---

## QNX 8.0 Build

Set up QNX SDP environment variables (`QNX_TARGET`, etc.) before building.

```bash
source ~/qnx800/qnxsdp-env.sh
# Optional (default: aarch64le)
export AUTOSAR_QNX_ARCH=aarch64le
```

FastDDS/CycloneDDS backends:

```bash
./build_libraries_qnx.sh <fastdds|cyclonedds> install
./build_data_types_qnx.sh <fastdds|cyclonedds> install
./build_lwrcl_qnx.sh <fastdds|cyclonedds> install
./build_apps_qnx.sh <fastdds|cyclonedds> install
```

Adaptive AUTOSAR backend (`adaptive-autosar`):

```bash
# 1) Build QNX middleware + AUTOSAR AP runtime (Adaptive-AUTOSAR repository)
cd ../Adaptive-AUTOSAR
./qnx/scripts/build_libraries_qnx.sh all install
./qnx/scripts/build_autosar_ap_qnx.sh install

# 2) Build lwrcl with adaptive-autosar backend
cd ../lwrcl-unified
./build_libraries_qnx.sh adaptive-autosar install
./build_data_types_qnx.sh adaptive-autosar install
./build_lwrcl_qnx.sh adaptive-autosar install
./build_apps_qnx.sh adaptive-autosar install
```

QNX default installation paths:

| Backend | Runtime/Middleware | lwrcl |
|---------|--------------------|-------|
| FastDDS | `/opt/qnx/fast-dds` | `/opt/qnx/fast-dds-libs` |
| CycloneDDS | `/opt/qnx/cyclonedds` (+ iceoryx: `/opt/qnx/iceoryx`) | `/opt/qnx/cyclonedds-libs` |
| Adaptive AUTOSAR | AUTOSAR AP: `/opt/qnx/autosar_ap/aarch64le`, CycloneDDS: `/opt/qnx/cyclonedds` | `/opt/qnx/autosar-ap-libs` (includes `yaml-cpp` via `build_libraries_qnx.sh adaptive-autosar`) |

---

## Repository Structure

```
lwrcl/
├── lwrcl/                      # Core library
│   ├── fastdds/               # FastDDS implementation
│   │   ├── lwrcl/            # lwrcl core
│   │   ├── tf2/              # Coordinate transforms
│   │   ├── tf2_ros/          # tf2 ROS integration
│   │   └── lwrcl_ffi/        # Dart/Flutter FFI
│   ├── cyclonedds/            # CycloneDDS implementation
│   │   ├── lwrcl/
│   │   ├── tf2/
│   │   └── tf2_ros/
│   ├── adaptive-autosar/       # Adaptive AUTOSAR (ara::com) implementation
│   │   ├── lwrcl/
│   │   ├── tf2/
│   │   └── tf2_ros/
│   └── vsomeip/               # vsomeip (SOME/IP) implementation
│       └── lwrcl/
├── data_types/                 # ROS 2 compatible message types
│   └── src/
│       ├── ros-data-types-for-fastdds/    # FastDDS (submodule)
│       └── ros-data-types-cyclonedds/     # CycloneDDS (submodule)
├── libraries/                  # Support libraries
│   └── src/
│       ├── yaml-cpp/          # YAML parser (submodule)
│       └── domain_participant_counter/
├── apps/                       # Sample applications
├── scripts/                    # DDS install & utility scripts
│   └── ...                     # Adaptive AUTOSAR codegen tools are provided by Adaptive-AUTOSAR project
├── packages/
│   └── lwrcl_dart/            # Dart/Flutter FFI bindings
├── build_libraries.sh
├── build_data_types.sh
├── build_lwrcl.sh
└── build_apps.sh
```

---

## Sample Applications

The following samples are available in `apps/lwrcl_example/`:

| Sample | Description |
|--------|-------------|
| `example_class_pub` | Class-based Publisher |
| `example_class_sub` | Class-based Subscriber |
| `example_class_pubsub_executor` | Pub/Sub with Executor |
| `example_namespace` | Namespace usage |
| `example_qos_presets` | QoS presets |
| `example_service_server` | Service server |
| `example_service_client` | Service client |
| `example_timer` | Timer usage |
| `example_timer_control` | Timer control |
| `example_spin` | spin / spin_some usage |
| `example_waitset` | WaitSet usage |
| `example_zero_copy_pub` | Zero Copy Publisher |
| `example_zero_copy_sub` | Zero Copy Subscriber |

Additional samples for image transport and custom message types are available under `apps/`.

---

## API Usage

lwrcl exposes its API under the `rclcpp` namespace. Below are basic usage examples.

### Initialization and Node Creation

```cpp
#include "rclcpp/rclcpp.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("my_node");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

### Publisher / Subscriber

```cpp
// Publisher
auto pub = node->create_publisher<std_msgs::msg::String>("topic", 10);
auto msg = std::make_shared<std_msgs::msg::String>();
msg->data() = "Hello";
pub->publish(msg);

// Subscriber
auto sub = node->create_subscription<std_msgs::msg::String>(
    "topic", 10,
    [](const std_msgs::msg::String& msg) {
        std::cout << msg.data() << std::endl;
    }
);
```

### Service / Client

```cpp
// Service
auto service = node->create_service<example_interfaces::srv::AddTwoInts>(
    "add_two_ints",
    [](std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> req,
       std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> res) {
        res->sum() = req->a() + req->b();
    }
);

// Client
auto client = node->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
```

### Timer

```cpp
auto timer = node->create_wall_timer(
    std::chrono::seconds(1),
    []() { std::cout << "tick" << std::endl; }
);
```

### QoS Configuration

```cpp
// Using a preset
auto pub = node->create_publisher<std_msgs::msg::String>("topic", rclcpp::SensorDataQoS());

// Custom QoS
rclcpp::QoS qos(10);
qos.reliability(rclcpp::QoS::ReliabilityPolicy::RELIABLE);
qos.durability(rclcpp::QoS::DurabilityPolicy::TRANSIENT_LOCAL);
auto pub2 = node->create_publisher<std_msgs::msg::String>("topic", qos);
```

### Parameter

```cpp
node->declare_parameter("my_param", 42);
int value;
node->get_parameter("my_param", value);
```

### Logging

```cpp
auto logger = node->get_logger();
RCLCPP_INFO(logger, "Hello %s", "world");
RCLCPP_WARN(logger, "Warning message");
RCLCPP_ERROR(logger, "Error: %d", error_code);
```

### Executor

```cpp
rclcpp::executors::MultiThreadedExecutor executor;
executor.add_node(node);
executor.spin();
```

---

## Dart / Flutter Integration (Experimental)

Dart FFI bindings are available in `packages/lwrcl_dart/`. The FFI build is disabled by default.

To enable it, set the `BUILD_FFI` environment variable:

```bash
BUILD_FFI=ON ./build_lwrcl.sh fastdds install
```

See `packages/lwrcl_dart/` for details.

---

## Related Projects

- [Fast-DDS](https://github.com/eProsima/Fast-DDS) — DDS implementation by eProsima
- [CycloneDDS](https://github.com/eclipse-cyclonedds/cyclonedds) — DDS implementation by Eclipse
- [vsomeip](https://github.com/COVESA/vsomeip) — SOME/IP implementation by COVESA
- [yaml-cpp](https://github.com/jbeder/yaml-cpp) — YAML parser
- [geometry2](https://github.com/ros2/geometry2) — Original tf2/tf2_ros project

---

## License

Apache License 2.0. See [LICENSE](LICENSE) for details.
