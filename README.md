# LWRCL (LightWeight Rclcpp Compatible Library) Fast DDS

**ROS 2 互換の軽量通信ライブラリ - QNX 8.0 サポート**

このライブラリは、ROS 2 の rclcpp と同様の API を提供しつつ、大幅に軽量化された DDS 通信ライブラリです。Fast DDS をバックエンドとして使用し、組み込みシステムやリソース制約のある環境でも ROS 2 エコシステムとシームレスに通信できます。

> **Note**: Cyclone DDS 版は [lwrcl-cyclonedds](https://github.com/tatsuyai713/lwrcl-cyclonedds) リポジトリをご参照ください。

---

## 🎯 lwrcl vs rclcpp 比較表

| 項目 | lwrcl (Fast DDS) | rclcpp (ROS 2) | 改善率 |
|------|------------------|----------------|--------|
| **コア行数** | ~5,100行 | ~50,000-80,000行 | **90-94% 削減** |
| **ソースファイル数** | 11ファイル | 180+ファイル | **94% 削減** |
| **依存パッケージ数** | 2パッケージ | 10+パッケージ | **80% 削減** |
| **ライブラリサイズ** | ~3.7 MB | 数百 MB | **大幅削減** |
| **ビルド時間** | 数分 | 数時間 | **大幅短縮** |
| **インストールサイズ** | 数十 MB | 数 GB | **大幅削減** |

### 機能対応表

| 機能 | lwrcl (Fast DDS) | rclcpp |
|------|:----------------:|:------:|
| **Node** | ✅ | ✅ |
| **Publisher / Subscription** | ✅ | ✅ |
| **Service / Client** | ✅ | ✅ |
| **Timer (create_timer / create_wall_timer)** | ✅ | ✅ |
| **Executor (Single / Multi-threaded)** | ✅ | ✅ |
| **QoS (Reliability, Durability, History)** | ✅ | ✅ |
| **QoS (Deadline, Lifespan, Liveliness)** | ✅ | ✅ |
| **QoS プリセット (SensorDataQoS, BestEffortQoS など)** | ✅ | ✅ |
| **Parameter (declare, get, set)** | ✅ | ✅ |
| **Parameter File (YAML)** | ✅ | ✅ |
| **Zero Copy (Loaned Messages)** | ✅ | ✅ |
| **WaitSet** | ✅ | ✅ |
| **Namespace サポート** | ✅ | ✅ |
| **Time / Duration / Clock / Rate** | ✅ | ✅ |
| **Logging (DEBUG, INFO, WARN, ERROR)** | ✅ | ✅ |
| **Serialization / Deserialization** | ✅ | ✅ |
| **tf2 / tf2_ros** | ✅ | ✅ |
| **rclcpp 互換 API** | ✅ | - |
| **Lifecycle Node** | ❌ | ✅ |
| **Action** | ❌ | ✅ |
| **Component** | ❌ | ✅ |
| **Topic Statistics** | ❌ | ✅ |
| **Intra-process 通信** | ✅ (DDS依存) | ✅ |

---

## 🚀 主な特徴

### 1. ROS 2 との完全互換通信
- ROS 2 ノードと直接トピック/サービス通信が可能
- 特別な設定なしでシームレスに連携

### 2. rclcpp 互換 API
- `rclcpp` 名前空間を使用して既存の ROS 2 コードを最小限の変更で移植可能
- `RCLCPP_INFO`, `RCLCPP_WARN` などのマクロも対応

### 3. 軽量・高速
- 依存関係を最小限に抑えた設計
- 組み込みデバイス（Raspberry Pi など）でも動作

### 4. マルチプラットフォーム
- **Linux** (Ubuntu/Debian)
- **QNX 8.0** (リアルタイムOS)

### 5. 高度な DDS 機能
- Zero Copy (Loaned Messages) による高性能通信
- 詳細な QoS 設定 (Deadline, Lifespan, Liveliness)
- WaitSet によるイベント駆動型プログラミング

---

## 📦 依存関係

| lwrcl | rclcpp (ROS 2) |
|-------|----------------|
| Fast DDS | rcl, rmw, rmw_implementation |
| yaml-cpp | rosidl_runtime, rosidl_typesupport |
| | rcutils, rcl_yaml_param_parser |
| | libstatistics_collector, tracing |
| **2 パッケージ** | **10+ パッケージ** |

---

## 🛠️ インストール方法

### 1. ROS 2 環境の無効化

`~/.bashrc` から ROS 2 の環境設定を削除してください：

```bash
# 以下の行をコメントアウトまたは削除
# source /opt/ros/humble/setup.bash
```

### 2. リポジトリのクローン

```bash
git clone --recursive https://github.com/tatsuyai713/lwrcl.git
cd lwrcl
```

### 3. Fast DDS のインストール

```bash
cd scripts
./install_fast_dds_ubuntu_debian.sh
source ~/.bashrc
```

### 4. サポートライブラリのビルド・インストール

```bash
cd ../lwrcl
./build_libraries.sh install
```

### 5. ROS データ型のビルド・インストール

```bash
./build_data_types.sh install
```

### 6. LWRCL のビルド・インストール

```bash
./build_lwrcl.sh install
```

### 7. サンプルアプリケーションのビルド

```bash
./build_apps.sh install
```

コンパイルされたアプリケーションは `apps/install` フォルダに格納されます。

---

## 📁 リポジトリ構成

```
lwrcl/
├── lwrcl/                    # Fast DDS 版 LWRCL
│   ├── lwrcl/               # コアライブラリ（lwrcl, tf2, tf2_ros）
│   ├── apps/                # サンプルアプリケーション
│   ├── data_types/          # ROS 2 互換データ型
│   └── libraries/           # サポートライブラリ
├── lwrcl-cyclonedds/        # Cyclone DDS 版（サブモジュール）
└── scripts/                 # Fast DDS インストールスクリプト
```

---

## 📖 API リファレンス

### Node の作成と管理

```cpp
#include "rclcpp/rclcpp.hpp"

// 初期化
rclcpp::init(argc, argv);

// ノード作成
auto node = rclcpp::Node::make_shared("my_node");
auto node_with_ns = rclcpp::Node::make_shared("my_node", "/my_namespace");

// ノード情報取得
std::string name = node->get_name();
std::string ns = node->get_namespace();
std::string fqn = node->get_fully_qualified_name();

// スピン
rclcpp::spin(node);
rclcpp::spin_some(node);

// シャットダウン
rclcpp::shutdown();
```

### Publisher

```cpp
// 作成
auto pub = node->create_publisher<std_msgs::msg::String>("topic", 10);
auto pub_qos = node->create_publisher<std_msgs::msg::String>("topic", rclcpp::SensorDataQoS());

// 通常のパブリッシュ
auto msg = std::make_shared<std_msgs::msg::String>();
msg->data() = "Hello";
pub->publish(msg);

// Zero Copy パブリッシュ
auto loaned_msg = pub->borrow_loaned_message();
loaned_msg->data() = "Zero Copy Hello";
pub->publish(std::move(loaned_msg));

// 購読者数の取得
int32_t count = pub->get_subscriber_count();
```

### Subscription

```cpp
// コールバック関数
void callback(std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(node->get_logger(), "Received: %s", msg->data().c_str());
}

// 作成
auto sub = node->create_subscription<std_msgs::msg::String>("topic", 10, callback);

// ラムダ式でも可
auto sub2 = node->create_subscription<std_msgs::msg::String>(
    "topic", 10,
    [](const std_msgs::msg::String& msg) {
        std::cout << msg.data() << std::endl;
    }
);

// 発行者数の取得
int32_t count = sub->get_publisher_count();
```

### Service / Client

```cpp
// サービスサーバー
auto service = node->create_service<example_interfaces::srv::AddTwoInts>(
    "add_two_ints",
    [](std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
       std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response) {
        response->sum() = request->a() + request->b();
    }
);

// サービスクライアント
auto client = node->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");

// サービス待機
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
// システムクロックタイマー
auto timer = node->create_timer(
    std::chrono::milliseconds(100),
    []() { std::cout << "Timer callback" << std::endl; }
);

// ウォールタイマー（モノトニッククロック）
auto wall_timer = node->create_wall_timer(
    std::chrono::seconds(1),
    []() { std::cout << "Wall timer callback" << std::endl; }
);
```

### QoS 設定

```cpp
// プリセット QoS
auto pub1 = node->create_publisher<T>("topic", rclcpp::SensorDataQoS());
auto pub2 = node->create_publisher<T>("topic", rclcpp::BestEffortQoS());
auto pub3 = node->create_publisher<T>("topic", rclcpp::ReliableQoS());

// カスタム QoS
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
            // メッセージ処理
        }
    } else if (result.kind() == rclcpp::WaitResultKind::Timeout) {
        RCLCPP_WARN(node->get_logger(), "Timeout");
    }
}
```

### Parameter

```cpp
// パラメータ宣言
node->declare_parameter("my_param", 42);
node->declare_parameter("my_string", "default_value");

// パラメータ取得
int value;
node->get_parameter("my_param", value);

std::string str_value;
node->get_parameter("my_string", str_value);

// 配列パラメータ
node->declare_parameter("my_array", std::vector<double>{1.0, 2.0, 3.0});
```

### Executor

```cpp
// シングルスレッド Executor
rclcpp::executors::SingleThreadedExecutor executor;
executor.add_node(node1);
executor.add_node(node2);
executor.spin();

// マルチスレッド Executor
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
    // 処理
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

## 📁 サンプルアプリケーション

| サンプル | 説明 |
|---------|------|
| `example_class_pub` | クラスベースの Publisher |
| `example_class_sub` | クラスベースの Subscriber |
| `example_class_pubsub_executor` | Executor を使用した Pub/Sub |
| `example_namespace` | Namespace の使用例 |
| `example_qos_presets` | QoS プリセットの使用例 |
| `example_service_server` | Service サーバー |
| `example_service_client` | Service クライアント |
| `example_timer` | Timer の使用例 |
| `example_timer_control` | Timer の制御 |
| `example_spin` | spin/spin_some の使用例 |
| `example_waitset` | WaitSet の使用例 |
| `example_zero_copy_pub` | Zero Copy Publisher |
| `example_zero_copy_sub` | Zero Copy Subscriber |

---

## 🔗 含まれるオープンソースプロジェクト

- [ROS Data Types](https://github.com/rticommunity/ros-data-types) - ROS 2 互換データ型
- [yaml-cpp](https://github.com/jbeder/yaml-cpp) - YAML パーサー
- [Fast-DDS](https://github.com/eProsima/Fast-DDS) - DDS 実装
- [geometry2](https://github.com/ros2/geometry2) - tf2/tf2_ros (座標変換ライブラリ)

---

## 📄 ライセンス

このプロジェクトはオープンソースライセンスの下で提供されています。詳細は [LICENSE](LICENSE) ファイルをご参照ください。
