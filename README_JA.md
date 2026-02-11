**[English (英語版)](README.md)**

# LWRCL (LightWeight Rclcpp Compatible Library)

**ROS 2 の rclcpp に似た API を持つ、軽量な DDS 通信ライブラリ**

lwrcl は、ROS 2 の rclcpp と互換性のある API を提供する軽量ライブラリです。DDS 実装として **CycloneDDS** と **FastDDS** の両方に対応しており、ROS 2 をフルインストールしなくても ROS 2 ノードとトピック・サービス通信を行えます。

---

## 特徴

- **rclcpp 互換 API** — `rclcpp` 名前空間を使用しているため、既存の ROS 2 コードを少ない変更で移植できます。`RCLCPP_INFO` / `RCLCPP_WARN` などのログマクロも利用可能です。
- **ROS 2 との相互通信** — ROS 2 ノードと同じ DDS ドメイン上でトピック・サービスを直接やり取りできます。
- **DDS バックエンド選択** — CycloneDDS と FastDDS をビルド時に選択できます。両方同時にインストールしておき、用途に応じて切り替えることも可能です。
- **依存関係が少ない** — DDS ライブラリと yaml-cpp のみに依存しており、ビルドが高速です。
- **マルチプラットフォーム** — Linux (Ubuntu/Debian)、QNX 8.0 に対応しています。
- **tf2 / tf2_ros 同梱** — 座標変換の基本機能を同梱しています。

---

## 対応する DDS 実装

| DDS 実装 | 特徴 |
|----------|------|
| **CycloneDDS** | Eclipse Foundation のオープンソース実装。軽量。 |
| **FastDDS** | eProsima 製。QoS オプションが豊富。 |

---

## 機能対応表

| 機能 | lwrcl | rclcpp |
|------|:-----:|:------:|
| Node | ✅ | ✅ |
| Publisher / Subscription | ✅ | ✅ |
| Service / Client | ✅ | ✅ |
| Timer (create_timer / create_wall_timer) | ✅ | ✅ |
| Executor (Single / Multi-threaded) | ✅ | ✅ |
| QoS (Reliability, Durability, History) | ✅ | ✅ |
| QoS (Deadline, Lifespan, Liveliness) | ✅ | ✅ |
| QoS プリセット (SensorDataQoS 等) | ✅ | ✅ |
| Parameter (declare, get, set) | ✅ | ✅ |
| Parameter File (YAML) | ✅ | ✅ |
| Zero Copy (Loaned Messages) | ✅ | ✅ |
| WaitSet | ✅ | ✅ |
| Namespace サポート | ✅ | ✅ |
| Time / Duration / Clock / Rate | ✅ | ✅ |
| Logging (DEBUG, INFO, WARN, ERROR) | ✅ | ✅ |
| Serialization / Deserialization | ✅ | ✅ |
| tf2 / tf2_ros | ✅ | ✅ |
| Lifecycle Node | ❌ | ✅ |
| Action | ❌ | ✅ |
| Component | ❌ | ✅ |
| Topic Statistics | ❌ | ✅ |
| Intra-process 通信 | ✅ (DDS 依存) | ✅ |

---

## 依存関係

- CMake 3.16.3 以上
- C++14 対応コンパイラ
- DDS 実装（CycloneDDS または FastDDS）
- yaml-cpp（サブモジュールとして同梱）

---

## ビルド手順 (Linux)

すべてのビルドスクリプトは以下の形式で実行します:

```
./build_<target>.sh <fastdds|cyclonedds> [install|clean]
```

- 第 1 引数: DDS バックエンド（`fastdds` または `cyclonedds`）
- 第 2 引数: `install` でビルド＆インストール、`clean` でビルドディレクトリ削除

### 前提条件

- Linux (Ubuntu 22.04 / 24.04 推奨)

> **Note**: ROS 2 がインストールされている場合は、`~/.bashrc` 等の `source /opt/ros/*/setup.bash` をコメントアウトしてください。lwrcl は ROS 2 に依存しませんが、同じ環境で ROS 2 の環境変数が設定されているとビルドが干渉する場合があります。

### 1. リポジトリのクローン

```bash
git clone --recursive <REPOSITORY_URL>
cd lwrcl
```

### 2. DDS のインストール

使用したい DDS 実装をインストールします。両方インストールしても問題ありません。

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

> Arch Linux の場合は `install_fast_dds_archlinux.sh` を使用してください。

### 3. サポートライブラリのビルド

```bash
./build_libraries.sh fastdds install
```

### 4. ROS データ型のビルド

ROS 2 互換のメッセージ型（`std_msgs`, `sensor_msgs`, `geometry_msgs` 等）をビルドします。

```bash
./build_data_types.sh fastdds install
```

### 5. lwrcl 本体のビルド

```bash
./build_lwrcl.sh fastdds install
```

### 6. サンプルアプリケーションのビルド（任意）

```bash
./build_apps.sh fastdds install
```

ビルド済みバイナリは `apps/install-fastdds/` に配置されます。

> **CycloneDDS を使う場合は、上記コマンドの `fastdds` を `cyclonedds` に置き換えてください。**

### ビルドディレクトリのクリーン

```bash
./build_lwrcl.sh fastdds clean
```

> ビルドディレクトリはバックエンドごとに分離されています（`build-fastdds` / `build-cyclonedds`）。バックエンドを切り替えても再クリーンは不要です。

### インストール先

| バックエンド | DDS インストール先 | lwrcl インストール先 |
|-------------|-------------------|---------------------|
| FastDDS | `/opt/fast-dds` | `/opt/fast-dds-libs` |
| CycloneDDS | `/opt/cyclonedds` | `/opt/cyclonedds-libs` |

---

## QNX 8.0 向けビルド

QNX SDP の環境変数（`QNX_TARGET` 等）を事前に設定してください。

```bash
./build_libraries_qnx.sh <fastdds|cyclonedds> install
./build_data_types_qnx.sh <fastdds|cyclonedds> install
./build_lwrcl_qnx.sh <fastdds|cyclonedds> install
./build_apps_qnx.sh <fastdds|cyclonedds> install
```

QNX でのインストール先:

| バックエンド | DDS | lwrcl |
|-------------|-----|-------|
| FastDDS | `/opt/qnx/fast-dds` | `/opt/qnx/fast-dds-libs` |
| CycloneDDS | `/opt/qnx/cyclonedds` | `/opt/qnx/cyclonedds-libs` |

---

## リポジトリ構成

```
lwrcl/
├── lwrcl/                      # コアライブラリ
│   ├── fastdds/               # FastDDS 版実装
│   │   ├── lwrcl/            # lwrcl 本体
│   │   ├── tf2/              # 座標変換ライブラリ
│   │   ├── tf2_ros/          # tf2 の ROS 連携
│   │   └── lwrcl_ffi/        # Dart/Flutter 用 FFI
│   └── cyclonedds/            # CycloneDDS 版実装
│       ├── lwrcl/
│       ├── tf2/
│       └── tf2_ros/
├── data_types/                 # ROS 2 互換メッセージ型
│   └── src/
│       ├── ros-data-types-for-fastdds/    # FastDDS 用 (submodule)
│       └── ros-data-types-cyclonedds/     # CycloneDDS 用 (submodule)
├── libraries/                  # サポートライブラリ
│   └── src/
│       ├── yaml-cpp/          # YAML パーサー (submodule)
│       └── domain_participant_counter/
├── apps/                       # サンプルアプリケーション
├── scripts/                    # DDS インストール・ユーティリティ
├── packages/
│   └── lwrcl_dart/            # Dart/Flutter FFI バインディング
├── build_libraries.sh
├── build_data_types.sh
├── build_lwrcl.sh
└── build_apps.sh
```

---

## サンプルアプリケーション

`apps/lwrcl_example/` に以下のサンプルが含まれています:

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
| `example_spin` | spin / spin_some の使用例 |
| `example_waitset` | WaitSet の使用例 |
| `example_zero_copy_pub` | Zero Copy Publisher |
| `example_zero_copy_sub` | Zero Copy Subscriber |

その他、画像転送やカスタムメッセージ型のサンプルも `apps/` 配下にあります。

---

## API の使い方

lwrcl は `rclcpp` 名前空間で API を提供します。以下は基本的な使い方です。

### 初期化とノード作成

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

### QoS 設定

```cpp
// プリセットを使う場合
auto pub = node->create_publisher<std_msgs::msg::String>("topic", rclcpp::SensorDataQoS());

// カスタム QoS
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

## Dart / Flutter 連携（実験的）

`packages/lwrcl_dart/` に Dart FFI バインディングがあります。FFI ビルドはデフォルトで無効です。

有効にするには `BUILD_FFI` 環境変数を設定してビルドします:

```bash
BUILD_FFI=ON ./build_lwrcl.sh fastdds install
```

詳細は `packages/lwrcl_dart/` を参照してください。

---

## 関連プロジェクト

- [Fast-DDS](https://github.com/eProsima/Fast-DDS) — eProsima の DDS 実装
- [CycloneDDS](https://github.com/eclipse-cyclonedds/cyclonedds) — Eclipse の DDS 実装
- [yaml-cpp](https://github.com/jbeder/yaml-cpp) — YAML パーサー
- [geometry2](https://github.com/ros2/geometry2) — tf2 / tf2_ros の元プロジェクト

---

## ライセンス

Apache License 2.0。詳細は [LICENSE](LICENSE) を参照してください。
