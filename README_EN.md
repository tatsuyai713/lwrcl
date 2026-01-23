# LWRCL (LightWeight Rclcpp Compatible Library) Fast DDS

**Lightweight ROS 2 Compatible Communication Library - QNX 8.0 Supported**

This library provides a significantly lighter DDS communication library with an API similar to ROS 2's rclcpp. Using Fast DDS as the backend, it enables seamless communication with the ROS 2 ecosystem even in embedded systems and resource-constrained environments.

> **Note**: For the Cyclone DDS version, please refer to the [lwrcl-cyclonedds](https://github.com/tatsuyai713/lwrcl-cyclonedds) repository.

---

## 🎯 lwrcl vs rclcpp Comparison

| Item | lwrcl (Fast DDS) | rclcpp (ROS 2) | Improvement |
|------|------------------|----------------|-------------|
| **Core Lines of Code** | ~5,100 lines | ~50,000-80,000 lines | **90-94% reduction** |
| **Source Files** | 11 files | 180+ files | **94% reduction** |
| **Dependency Packages** | 2 packages | 10+ packages | **80% reduction** |
| **Library Size** | ~3.7 MB | Hundreds of MB | **Significant reduction** |
| **Build Time** | Minutes | Hours | **Significant reduction** |
| **Install Size** | Tens of MB | Several GB | **Significant reduction** |

### Feature Support Matrix

| Feature | lwrcl (Fast DDS) | rclcpp |
|---------|:----------------:|:------:|
| **Node** | ✅ | ✅ |
| **Publisher / Subscription** | ✅ | ✅ |
| **Service / Client** | ✅ | ✅ |
| **Timer (create_timer / create_wall_timer)** | ✅ | ✅ |
| **Executor (Single / Multi-threaded)** | ✅ | ✅ |
| **QoS (Reliability, Durability, History)** | ✅ | ✅ |
| **QoS (Deadline, Lifespan, Liveliness)** | ✅ | ✅ |
| **QoS Presets (SensorDataQoS, BestEffortQoS, etc.)** | ✅ | ✅ |
| **Parameter (declare, get, set)** | ✅ | ✅ |
| **Parameter File (YAML)** | ✅ | ✅ |
| **Zero Copy (Loaned Messages)** | ✅ | ✅ |
| **WaitSet** | ✅ | ✅ |
| **Namespace Support** | ✅ | ✅ |
| **Time / Duration / Clock / Rate** | ✅ | ✅ |
| **Logging (DEBUG, INFO, WARN, ERROR)** | ✅ | ✅ |
| **Serialization / Deserialization** | ✅ | ✅ |
| **tf2 / tf2_ros** | ✅ | ✅ |
| **rclcpp Compatible API** | ✅ | - |
| **Lifecycle Node** | ❌ | ✅ |
| **Action** | ❌ | ✅ |
| **Component** | ❌ | ✅ |
| **Topic Statistics** | ❌ | ✅ |
| **Intra-process Communication** | ✅ (DDS dependent) | ✅ |

---

## 🚀 Key Features

### 1. Full ROS 2 Communication Compatibility
- Direct topic/service communication with ROS 2 nodes
- Seamless integration without special configuration

### 2. rclcpp Compatible API
- Use the `rclcpp` namespace to port existing ROS 2 code with minimal changes
- Macros like `RCLCPP_INFO`, `RCLCPP_WARN` are also supported

### 3. Lightweight & Fast
- Designed with minimal dependencies
- Runs on embedded devices (e.g., Raspberry Pi)

### 4. Multi-platform Support
- **Linux** (Ubuntu/Debian)
- **QNX 8.0** (Real-time OS)

### 5. Advanced DDS Features
- High-performance communication via Zero Copy (Loaned Messages)
- Detailed QoS settings (Deadline, Lifespan, Liveliness)
- Event-driven programming with WaitSet

---

## 📦 Dependencies

| lwrcl | rclcpp (ROS 2) |
|-------|----------------|
| Fast DDS | rcl, rmw, rmw_implementation |
| yaml-cpp | rosidl_runtime, rosidl_typesupport |
| | rcutils, rcl_yaml_param_parser |
| | libstatistics_collector, tracing |
| **2 packages** | **10+ packages** |

---

## 🛠️ Installation

### 1. Disable ROS 2 Environment

Remove or comment out the ROS 2 environment setup in `~/.bashrc`:

```bash
# Comment out or remove the following line
# source /opt/ros/humble/setup.bash
```

### 2. Clone the Repository

```bash
git clone --recursive https://github.com/tatsuyai713/lwrcl.git
cd lwrcl
```

### 3. Install Fast DDS

```bash
cd scripts
./install_fast_dds_ubuntu_debian.sh
source ~/.bashrc
```

### 4. Build and Install Support Libraries

```bash
cd ../lwrcl
./build_libraries.sh install
```

### 5. Build and Install ROS Data Types

```bash
./build_data_types.sh install
```

### 6. Build and Install LWRCL

```bash
./build_lwrcl.sh install
```

### 7. Build Sample Applications

```bash
./build_apps.sh install
```

Compiled applications will be placed in the `apps/install` folder.

---

## 📁 Repository Structure

```
lwrcl/
├── lwrcl/                    # Fast DDS version of LWRCL
│   ├── lwrcl/               # Core library (lwrcl, tf2, tf2_ros)
│   ├── apps/                # Sample applications
│   ├── data_types/          # ROS 2 compatible data types
│   └── libraries/           # Support libraries
├── lwrcl-cyclonedds/        # Cyclone DDS version (submodule)
└── scripts/                 # Fast DDS installation scripts
```

---

## 📖 API Reference

### Node Creation and Management

```cpp
#include "rclcpp/rclcpp.hpp"

// Initialization
rclcpp::init(argc, argv);

// Create node
auto node = rclcpp::Node::make_shared("my_node");
auto node_with_ns = rclcpp::Node::make_shared("my_node", "/my_namespace");

// Get node info
std::string name = node->get_name();
std::string ns = node->get_namespace();
std::string fqn = node->get_fully_qualified_name();

// Spin
rclcpp::spin(node);
rclcpp::spin_some(node);

// Shutdown
rclcpp::shutdown();
```

### Publisher

```cpp
// Create
auto pub = node->create_publisher<std_msgs::msg::String>("topic", 10);
auto pub_qos = node->create_publisher<std_msgs::msg::String>("topic", rclcpp::SensorDataQoS());

// Normal publish
auto msg = std::make_shared<std_msgs::msg::String>();
msg->data() = "Hello";
pub->publish(msg);

// Zero Copy publish
auto loaned_msg = pub->borrow_loaned_message();
loaned_msg->data() = "Zero Copy Hello";
pub->publish(std::move(loaned_msg));

// Get subscriber count
int32_t count = pub->get_subscriber_count();
```

### Subscription

```cpp
// Callback function
void callback(std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(node->get_logger(), "Received: %s", msg->data().c_str());
}

// Create
auto sub = node->create_subscription<std_msgs::msg::String>("topic", 10, callback);

// Lambda expression also works
auto sub2 = node->create_subscription<std_msgs::msg::String>(
    "topic", 10,
    [](const std_msgs::msg::String& msg) {
        std::cout << msg.data() << std::endl;
    }
);

// Get publisher count
int32_t count = sub->get_publisher_count();
```

### Service / Client

```cpp
// Service server
auto service = node->create_service<example_interfaces::srv::AddTwoInts>(
    "add_two_ints",
    [](std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
       std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response) {
        response->sum() = request->a() + request->b();
    }
);

// Service client
auto client = node->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");

// Wait for service
if (client->wait_for_service(std::chrono::seconds(5))) {
    auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
    request->a() = 1;
    request->b() = 2;
    auto future = client->async_send_request(request);
    rclcpp::spin_until_future_complete(node, future, std::chrono::seconds(10));
}
```

### Timer

```cpp
// System clock timer
auto timer = node->create_timer(
    std::chrono::milliseconds(100),
    []() { std::cout << "Timer callback" << std::endl; }
);

// Wall timer (monotonic clock)
auto wall_timer = node->create_wall_timer(
    std::chrono::seconds(1),
    []() { std::cout << "Wall timer callback" << std::endl; }
);
```

### QoS Configuration

```cpp
// Preset QoS
auto pub1 = node->create_publisher<T>("topic", rclcpp::SensorDataQoS());
auto pub2 = node->create_publisher<T>("topic", rclcpp::BestEffortQoS());
auto pub3 = node->create_publisher<T>("topic", rclcpp::ReliableQoS());

// Custom QoS
rclcpp::QoS qos(10);
qos.reliability(rclcpp::QoS::ReliabilityPolicy::RELIABLE);
qos.durability(rclcpp::QoS::DurabilityPolicy::TRANSIENT_LOCAL);
qos.deadline(std::chrono::milliseconds(100));
qos.lifespan(std::chrono::seconds(5));
qos.liveliness(rclcpp::QoS::LivelinessPolicy::AUTOMATIC);
auto pub4 = node->create_publisher<T>("topic", qos);
```

### WaitSet

```cpp
rclcpp::WaitSet wait_set({{subscription}});

while (rclcpp::ok()) {
    auto result = wait_set.wait(std::chrono::seconds(1));
    if (result.kind() == rclcpp::WaitResultKind::Ready) {
        MyMessage msg;
        rclcpp::MessageInfo info;
        if (subscription->take(msg, info)) {
            // Process message
        }
    } else if (result.kind() == rclcpp::WaitResultKind::Timeout) {
        RCLCPP_WARN(node->get_logger(), "Timeout");
    }
}
```

### Parameter

```cpp
// Declare parameter
node->declare_parameter("my_param", 42);
node->declare_parameter("my_string", "default_value");

// Get parameter
int value;
node->get_parameter("my_param", value);

std::string str_value;
node->get_parameter("my_string", str_value);

// Array parameter
node->declare_parameter("my_array", std::vector<double>{1.0, 2.0, 3.0});
```

### Executor

```cpp
// Single-threaded Executor
rclcpp::executors::SingleThreadedExecutor executor;
executor.add_node(node1);
executor.add_node(node2);
executor.spin();

// Multi-threaded Executor
rclcpp::executors::MultiThreadedExecutor mt_executor;
mt_executor.add_node(node1);
mt_executor.add_node(node2);
mt_executor.spin();
```

### Time / Duration / Rate

```cpp
// Time
rclcpp::Time now = node->get_clock()->now();

// Duration
rclcpp::Duration duration(std::chrono::seconds(1));
int64_t ns = duration.nanoseconds();
double sec = duration.seconds();

// Rate
rclcpp::Rate rate(rclcpp::Duration(std::chrono::milliseconds(100)));
while (rclcpp::ok()) {
    // Processing
    rate.sleep();
}

// WallRate
rclcpp::WallRate wall_rate(rclcpp::Duration(std::chrono::milliseconds(100)));
```

### Logging

```cpp
auto logger = node->get_logger();
RCLCPP_DEBUG(logger, "Debug message");
RCLCPP_INFO(logger, "Info: %d", value);
RCLCPP_WARN(logger, "Warning message");
RCLCPP_ERROR(logger, "Error: %s", error_msg.c_str());
```

---

## 📁 Sample Applications

| Sample | Description |
|--------|-------------|
| `example_class_pub` | Class-based Publisher |
| `example_class_sub` | Class-based Subscriber |
| `example_class_pubsub_executor` | Pub/Sub with Executor |
| `example_namespace` | Namespace usage example |
| `example_qos_presets` | QoS presets usage example |
| `example_service_server` | Service server |
| `example_service_client` | Service client |
| `example_timer` | Timer usage example |
| `example_timer_control` | Timer control |
| `example_spin` | spin/spin_some usage example |
| `example_waitset` | WaitSet usage example |
| `example_zero_copy_pub` | Zero Copy Publisher |
| `example_zero_copy_sub` | Zero Copy Subscriber |

---

## 🔗 Included Open Source Projects

- [ROS Data Types](https://github.com/rticommunity/ros-data-types) - ROS 2 compatible data types
- [yaml-cpp](https://github.com/jbeder/yaml-cpp) - YAML parser
- [Fast-DDS](https://github.com/eProsima/Fast-DDS) - DDS implementation
- [geometry2](https://github.com/ros2/geometry2) - tf2/tf2_ros (coordinate transformation library)

---

## 📄 License

This project is provided under an open source license. Please refer to the [LICENSE](LICENSE) file for details.
