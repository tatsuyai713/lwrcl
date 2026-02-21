[![FastDDS CI](https://github.com/tatsuyai713/lwrcl/actions/workflows/ci-fastdds.yml/badge.svg)](https://github.com/tatsuyai713/lwrcl/actions/workflows/ci-fastdds.yml)
[![CycloneDDS CI](https://github.com/tatsuyai713/lwrcl/actions/workflows/ci-cyclonedds.yml/badge.svg)](https://github.com/tatsuyai713/lwrcl/actions/workflows/ci-cyclonedds.yml)
[![vsomeip CI](https://github.com/tatsuyai713/lwrcl/actions/workflows/ci-vsomeip.yml/badge.svg)](https://github.com/tatsuyai713/lwrcl/actions/workflows/ci-vsomeip.yml)
[![Adaptive AUTOSAR CI](https://github.com/tatsuyai713/lwrcl/actions/workflows/ci-adaptive-autosar.yml/badge.svg)](https://github.com/tatsuyai713/lwrcl/actions/workflows/ci-adaptive-autosar.yml)

**[English (英語版)](README.md)**

# LWRCL (LightWeight Rclcpp Compatible Library)

**ROS 2 の rclcpp に似た API を持つ、軽量な DDS 通信ライブラリ**

lwrcl は、ROS 2 の rclcpp と互換性のある API を提供する軽量ライブラリです。通信バックエンドとして **CycloneDDS**、**FastDDS**、**vsomeip (SOME/IP)**、**Adaptive AUTOSAR (ara::com)** に対応しており、ROS 2 をフルインストールしなくても ROS 2 ノードとトピック・サービス通信を行えます。

---

## 特徴

- **rclcpp 互換 API** — `rclcpp` 名前空間を使用しているため、既存の ROS 2 コードを少ない変更で移植できます。`RCLCPP_INFO` / `RCLCPP_WARN` などのログマクロも利用可能です。
- **ROS 2 との相互通信** — ROS 2 ノードと同じ DDS ドメイン上でトピック・サービスを直接やり取りできます。
- **DDS バックエンド選択** — CycloneDDS、FastDDS、vsomeip、Adaptive AUTOSAR をビルド時に選択できます。複数同時にインストールしておき、用途に応じて切り替えることも可能です。
- **依存関係が少ない** — DDS ライブラリと yaml-cpp のみに依存しており、ビルドが高速です。
- **マルチプラットフォーム** — Linux (Ubuntu/Debian)、QNX 8.0 に対応しています。
- **tf2 / tf2_ros 同梱** — 座標変換の基本機能を同梱しています。

---

## 対応する DDS 実装

| DDS 実装 | 特徴 |
|----------|------|
| **CycloneDDS** | Eclipse Foundation のオープンソース実装。軽量。 |
| **FastDDS** | eProsima 製。QoS オプションが豊富。 |
| **vsomeip** | COVESA の SOME/IP 実装。DDS ランタイム不要の車載グレード通信。 |
| **Adaptive AUTOSAR** | `ara::com` ベースのバックエンド（Adaptive-AUTOSAR 連携）。 |

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
- 通信バックエンド（CycloneDDS、FastDDS、vsomeip、または Adaptive AUTOSAR）
- yaml-cpp（サブモジュールとして同梱）
- Boost（vsomeip バックエンド使用時に必要）

---

## ビルド手順 (Linux)

すべてのビルドスクリプトは以下の形式で実行します:

```
./build_<target>.sh <fastdds|cyclonedds|vsomeip|adaptive-autosar> [install|clean]
```

- 第 1 引数: 通信バックエンド（`fastdds`、`cyclonedds`、`vsomeip`、または `adaptive-autosar`）
- 第 2 引数: `install` でビルド＆インストール、`clean` でビルドディレクトリ削除

### 前提条件

- Linux (Ubuntu 22.04 / 24.04 推奨)

> **Note**: ROS 2 がインストールされている場合は、`~/.bashrc` 等の `source /opt/ros/*/setup.bash` をコメントアウトしてください。lwrcl は ROS 2 に依存しませんが、同じ環境で ROS 2 の環境変数が設定されているとビルドが干渉する場合があります。

### 1. リポジトリのクローン

```bash
git clone --recursive <REPOSITORY_URL>
cd lwrcl
```

### 2. 通信バックエンドのインストール

使用したいバックエンドをインストールします。複数インストールしても問題ありません。

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

> Arch Linux の場合は `install_fast_dds_archlinux.sh` を使用してください。

**vsomeip (SOME/IP):**

```bash
./scripts/install_vsomeip.sh
```

> vsomeip は Boost と CycloneDDS（ビルド時の `idlc` コード生成ツール用のみ）が必要です。先に CycloneDDS をインストールしてから `install_vsomeip.sh` を実行してください。vsomeip バックエンドは DDS ランタイムに**依存しません** — cyclonedds-cxx から抽出した CDR シリアライゼーションをスタンドアロンの静的ライブラリとして使用します。

**Adaptive AUTOSAR (ara::com):**

- Adaptive AUTOSAR AP ランタイムを `/opt/autosar_ap` に導入します（例: [`Adaptive-AUTOSAR`](https://github.com/tatsuyai713/Adaptive-AUTOSAR)）。
- `ara::com` の内部DDSとして CycloneDDS も必要です。
- ビルドは `adaptive-autosar` バックエンドで実行します（下記）。

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
> **vsomeip を使う場合は、上記コマンドの `fastdds` を `vsomeip` に置き換えてください。**
> **Adaptive AUTOSAR の場合は、以下の専用手順を実行してください。**

### Adaptive AUTOSAR バックエンドのビルド手順

Adaptive AUTOSAR バックエンドは `ara::com` API を使用しますが、`yaml-cpp` は必要です。最初に `build_libraries.sh adaptive-autosar install` を実行してください。

```bash
./build_libraries.sh adaptive-autosar install
./build_data_types.sh adaptive-autosar install
./build_lwrcl.sh adaptive-autosar install
./build_apps.sh adaptive-autosar install
```

ビルド済みバイナリは `apps/install-adaptive-autosar/` に配置されます。

### ビルドディレクトリのクリーン

```bash
./build_lwrcl.sh fastdds clean
```

> ビルドディレクトリはバックエンドごとに分離されています（`build-fastdds` / `build-cyclonedds` / `build-vsomeip` / `build-adaptive-autosar`）。バックエンドを切り替えても再クリーンは不要です。

### インストール先

| バックエンド | DDS インストール先 | lwrcl インストール先 |
|-------------|-------------------|---------------------|
| FastDDS | `/opt/fast-dds` | `/opt/fast-dds-libs` |
| CycloneDDS | `/opt/cyclonedds` | `/opt/cyclonedds-libs` |
| vsomeip | `/opt/vsomeip` | `/opt/vsomeip-libs` |
| Adaptive AUTOSAR | `/opt/autosar_ap` | `/opt/autosar-ap-libs`（`build_libraries.sh adaptive-autosar` で `yaml-cpp` も導入） |

---

## CycloneDDS の Zero Copy (iceoryx)

CycloneDDS バックエンドでは、lwrcl は利用可能な場合に native の writer-loan/read-loan API を使用し、利用できない場合は安全にフォールバックします。

iceoryx SHM 転送によるゼロコピーを有効化するには:

```bash
./scripts/install_iceoryx.sh
./scripts/install_cyclonedds.sh --enable-shm
export LD_LIBRARY_PATH=/opt/iceoryx/lib:/opt/cyclonedds/lib:/opt/cyclonedds-libs/lib:${LD_LIBRARY_PATH}
export CYCLONEDDS_URI=file:///opt/cyclonedds/etc/cyclonedds-lwrcl.xml
iox-roudi
```

`install_iceoryx.sh` は、`/dev/shm` が ACL 非対応のコンテナ環境でも起動できるように、デフォルトで ACL フォールバックを適用します。  
厳密な ACL 強制が必要な場合は、以下で再インストールしてください。

```bash
./scripts/install_iceoryx.sh --force --strict-acl
```

コンテナ設定で対応する場合（ホストが tmpfs ACL 対応なら推奨）:

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

ホストカーネル/ファイルシステム側が tmpfs の POSIX ACL をサポートしていない場合、コンテナ設定だけでは ACL を有効化できません。

その後 `cyclonedds` バックエンドでビルドし、`example_zero_copy_pub` / `example_zero_copy_sub` を実行してください。

---

## QNX 8.0 向けビルド

QNX SDP の環境変数（`QNX_TARGET` 等）を事前に設定してください。

```bash
source ~/qnx800/qnxsdp-env.sh
# 任意（デフォルト: aarch64le）
export AUTOSAR_QNX_ARCH=aarch64le
```

FastDDS/CycloneDDS バックエンド:

```bash
./build_libraries_qnx.sh <fastdds|cyclonedds> install
./build_data_types_qnx.sh <fastdds|cyclonedds> install
./build_lwrcl_qnx.sh <fastdds|cyclonedds> install
./build_apps_qnx.sh <fastdds|cyclonedds> install
```

Adaptive AUTOSAR バックエンド（`adaptive-autosar`）:

```bash
# 1) QNX向けミドルウェア + AUTOSAR AP ランタイムをビルド（Adaptive-AUTOSAR リポジトリ）
cd ../Adaptive-AUTOSAR
./qnx/scripts/build_libraries_qnx.sh all install
./qnx/scripts/build_autosar_ap_qnx.sh install

# 2) lwrcl を adaptive-autosar でビルド
cd ../lwrcl-unified
./build_libraries_qnx.sh adaptive-autosar install
./build_data_types_qnx.sh adaptive-autosar install
./build_lwrcl_qnx.sh adaptive-autosar install
./build_apps_qnx.sh adaptive-autosar install
```

QNX の既定インストール先:

| バックエンド | ランタイム/ミドルウェア | lwrcl |
|-------------|-------------------------|-------|
| FastDDS | `/opt/qnx/fast-dds` | `/opt/qnx/fast-dds-libs` |
| CycloneDDS | `/opt/qnx/cyclonedds`（+ iceoryx: `/opt/qnx/iceoryx`） | `/opt/qnx/cyclonedds-libs` |
| Adaptive AUTOSAR | AUTOSAR AP: `/opt/qnx/autosar_ap/aarch64le`、CycloneDDS: `/opt/qnx/cyclonedds` | `/opt/qnx/autosar-ap-libs`（`build_libraries_qnx.sh adaptive-autosar` で `yaml-cpp` も導入） |

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
│   ├── cyclonedds/            # CycloneDDS 版実装
│   │   ├── lwrcl/
│   │   ├── tf2/
│   │   └── tf2_ros/
│   ├── adaptive-autosar/       # Adaptive AUTOSAR (ara::com) 版実装
│   │   ├── lwrcl/
│   │   ├── tf2/
│   │   └── tf2_ros/
│   └── vsomeip/               # vsomeip (SOME/IP) 版実装
│       └── lwrcl/
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
- [vsomeip](https://github.com/COVESA/vsomeip) — COVESA の SOME/IP 実装
- [yaml-cpp](https://github.com/jbeder/yaml-cpp) — YAML パーサー
- [geometry2](https://github.com/ros2/geometry2) — tf2 / tf2_ros の元プロジェクト

---

## ライセンス

Apache License 2.0。詳細は [LICENSE](LICENSE) を参照してください。
