# LWRCL (LightWeight Rclcpp Compatible Library)

**A lightweight DDS communication library with an rclcpp-compatible API**

lwrcl provides an API compatible with ROS 2's rclcpp. It supports both **CycloneDDS** and **FastDDS** as DDS backends, enabling direct topic and service communication with ROS 2 nodes without a full ROS 2 installation.

---

## Features

- **rclcpp-compatible API** — Uses the `rclcpp` namespace, allowing existing ROS 2 code to be ported with minimal changes. Logging macros such as `RCLCPP_INFO` and `RCLCPP_WARN` are also available.
- **Interoperability with ROS 2** — Communicates directly with ROS 2 nodes via topics and services on the same DDS domain.
- **DDS backend selection** — Choose between CycloneDDS and FastDDS at build time. Both can be installed simultaneously and switched as needed.
- **Minimal dependencies** — Depends only on a DDS library and yaml-cpp, resulting in fast builds.
- **Multi-platform** — Supports Linux (Ubuntu/Debian) and QNX 8.0.
- **Includes tf2 / tf2_ros** — Bundled coordinate transformation support.

---

## Supported DDS Implementations

| DDS Implementation | Description |
|-------------------|-------------|
| **CycloneDDS** | Eclipse Foundation open-source implementation. Lightweight. |
| **FastDDS** | By eProsima. Rich QoS options. |

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
- DDS implementation (CycloneDDS or FastDDS)
- yaml-cpp (included as a submodule)

---

## Build Instructions (Linux)

All build scripts follow this pattern:

```
./build_<target>.sh <fastdds|cyclonedds> [install|clean]
```

- 1st argument: DDS backend (`fastdds` or `cyclonedds`)
- 2nd argument: `install` to build & install, `clean` to remove build directory

### Prerequisites

- Linux (Ubuntu 22.04 / 24.04 recommended)

> **Note**: If ROS 2 is installed, comment out `source /opt/ros/*/setup.bash` in your `~/.bashrc`. lwrcl does not depend on ROS 2, but having ROS 2 environment variables set may cause build conflicts.

### 1. Clone the Repository

```bash
git clone --recursive <REPOSITORY_URL>
cd lwrcl
```

### 2. Install DDS

Install the DDS implementation you want to use. You may install both without issues.

**CycloneDDS:**

```bash
./scripts/install_cyclone_dds.sh
source ~/.bashrc
```

**FastDDS (Ubuntu/Debian):**

```bash
./scripts/install_fast_dds_ubuntu_debian.sh
source ~/.bashrc
```

> For Arch Linux, use `install_fast_dds_archlinux.sh` instead.

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

### Cleaning Build Directories

```bash
./build_lwrcl.sh fastdds clean
```

> Build directories are separated by backend (`build-fastdds` / `build-cyclonedds`). Switching backends does not require cleaning.

### Installation Paths

| Backend | DDS Install Path | lwrcl Install Path |
|---------|-----------------|-------------------|
| FastDDS | `/opt/fast-dds` | `/opt/fast-dds-libs` |
| CycloneDDS | `/opt/cyclonedds` | `/opt/cyclonedds-libs` |

---

## QNX 8.0 Build

Set up QNX SDP environment variables (`QNX_TARGET`, etc.) before building.

```bash
./build_libraries_qnx.sh <fastdds|cyclonedds> install
./build_data_types_qnx.sh <fastdds|cyclonedds> install
./build_lwrcl_qnx.sh <fastdds|cyclonedds> install
./build_apps_qnx.sh <fastdds|cyclonedds> install
```

QNX installation paths:

| Backend | DDS | lwrcl |
|---------|-----|-------|
| FastDDS | `/opt/qnx/fast-dds` | `/opt/qnx/fast-dds-libs` |
| CycloneDDS | `/opt/qnx/cyclonedds` | `/opt/qnx/cyclonedds-libs` |

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
│   └── cyclonedds/            # CycloneDDS implementation
│       ├── lwrcl/
│       ├── tf2/
│       └── tf2_ros/
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
- [yaml-cpp](https://github.com/jbeder/yaml-cpp) — YAML parser
- [geometry2](https://github.com/ros2/geometry2) — Original tf2/tf2_ros project

---

## License

Apache License 2.0. See [LICENSE](LICENSE) for details.
