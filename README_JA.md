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

## パフォーマンスベンチマーク

以下の数値はすべて **x86\_64 Linux ホスト**（Docker コンテナ、Ubuntu 22.04）上で CycloneDDS 0.10.5 を使用して計測しました。
レイテンシおよびスループットの数値は **CycloneDDS `ddsperf`** のプロセス内（`-L`）モードを使用しており、
ネットワークの影響を排除した状態で計測しています。これは lwrcl があらゆるバックエンドで達成できる下限値を示します。

### 1. ライブラリ・バイナリサイズ

| コンポーネント | ファイル | サイズ（デバッグ情報付き） | サイズ（ストリップ後） |
|-------------|--------|-------------------:|----------------:|
| lwrcl ラッパー | `liblwrcl.so` (CycloneDDS) | 3.1 MB | **811 KB** |
| lwrcl ラッパー | `liblwrcl.so` (FastDDS) | 3.7 MB | **1.1 MB** |
| lwrcl ラッパー | `liblwrcl.so` (vsomeip) | 2.7 MB | **738 KB** |
| CycloneDDS ランタイム | `libddsc.so` | — | 1.6 MB |
| CycloneDDS C++ バインディング | `libddscxx.so` | — | 1.4 MB |
| FastDDS ランタイム | `libfastrtps.so` | — | 12 MB |
| アプリケーションバイナリ | `example_class_pub` (CycloneDDS) | 208 KB | **143 KB** |
| アプリケーションバイナリ | `example_class_pub` (vsomeip) | 943 KB | 143 KB |

> lwrcl ラッパー本体（`liblwrcl.so`）はストリップ後わずか **約 800 KB** — DDS ランタイムの薄いラッパー層です。
> パブリッシャー/サブスクライバーアプリケーションのバイナリは **ストリップ後 150 KB 未満** です。

### 2. 実行時メモリフットプリント

起動直後の単一サブスクライバーノード（`example_class_sub`、CycloneDDS バックエンド）で計測：

| 指標 | 値 |
|------|---:|
| VmRSS（常駐セットサイズ） | **~15 MB** |
| VmSize（仮想メモリ） | ~635 MB\* |

\* 仮想メモリが大きいのは DDS 共有メモリ領域の予約によるもので、実際の物理メモリ使用量は RSS の値が示します。

### 3. エンドツーエンドレイテンシ

`ddsperf -L ping pong`（プロセス内ピンポン、CycloneDDS 0.10.5、Release ビルド）で計測。
数値は**往復遅延 ÷ 2**（片道レイテンシ）です。

| メッセージサイズ | 平均 | p50 | p90 | p99 | 最小 |
|--------------|----:|----:|----:|----:|----:|
| 12 バイト (KS) | 13.9 µs | 13.0 µs | 17.1 µs | 21 µs | 5 µs |
| 32 バイト (K32) | 13.4 µs | 12.8 µs | 16.7 µs | 19 µs | 4 µs |
| 256 バイト (K256) | 14.2 µs | 13.2 µs | 17.3 µs | 19 µs | 1 µs |

全ての代表的なメッセージサイズで p90 レイテンシ 20 µs 未満を達成。

### 4. スループット

`ddsperf -L pub sub`（プロセス内パブリッシャー＋サブスクライバー、CycloneDDS 0.10.5、Release ビルド）で計測。

| メッセージサイズ | メッセージ数/秒 | 帯域幅 |
|--------------|-------------:|------:|
| 12 バイト (KS) | ~210 万/秒 | ~201 Mb/s |
| 32 バイト (K32) | ~240 万/秒 | ~613 Mb/s |
| 256 バイト (K256) | ~210 万/秒 | **~4.3 Gb/s** |

全ての実行でパケットロスなし。

### 5. ビルド時間

ソースからの lwrcl ライブラリフルビルド（Release、`$(nproc)` 並列ジョブ、コールドキャッシュ）：

| バックエンド | 設定 | コンパイル | 合計 |
|-----------|----:|--------:|-----:|
| CycloneDDS | 1.7 秒 | ~15 秒 | **~17 秒** |
| FastDDS | 2.0 秒 | ~15 秒 | **~18 秒** |

> lwrcl ライブラリ全体のビルドは、最新のマシンで **20 秒未満**で完了します。

### 6. ソースコード量

バックエンドごとの C++ ソースコード総行数（`.cpp` ＋ `.hpp`）：

| バックエンド | 行数（ヘッダー＋ソース） |
|-----------|---------------------:|
| CycloneDDS | ~5,200 |
| FastDDS | ~5,400 |
| vsomeip | ~4,500 |
| Adaptive AUTOSAR | ~4,800 |

コードベース全体を午後一日で手動監査できる規模です。

---

### ベンチマークの再現方法

**レイテンシ（ピンポン）:**
```bash
LD_LIBRARY_PATH=/opt/iceoryx/lib:/opt/cyclonedds/lib \
  ddsperf -L -D 10 ping pong
```

**スループット:**
```bash
LD_LIBRARY_PATH=/opt/iceoryx/lib:/opt/cyclonedds/lib \
  ddsperf -L -D 10 -T K256 pub sub
```

**メモリフットプリント:**
```bash
LD_LIBRARY_PATH=/opt/iceoryx/lib:/opt/cyclonedds/lib:/opt/cyclonedds-libs/lib \
  example_class_sub &
grep VmRSS /proc/$!/status
```

---

## ROS2 Humble との比較

以下の表は lwrcl（CycloneDDS バックエンド）と **ROS2 Humble**（デフォルト rmw_fastrtps）を比較したものです。
すべての計測は同一マシン（x86\_64、Ubuntu 22.04、Docker コンテナ）上で行いました。
ROS2 のレイテンシは同一プロセス内の C++ rclcpp パブリッシャー/サブスクライバーペアで計測しており
（ddsperf の `-L` プロセス内モードと同等）、メッセージ型は `std_msgs::msg::String` を使用しています。

### ミドルウェア層のサイズ

| コンポーネント | lwrcl + CycloneDDS | ROS2 Humble + FastDDS |
|-------------|-------------------:|----------------------:|
| ラッパー / rclcpp 層 | **811 KB**（liblwrcl.so ストリップ後） | 3.0 MB（rclcpp + rcl + rmw スタック） |
| DDS ランタイム | 3.0 MB（libddsc + libddscxx） | 12 MB（libfastrtps.so） |
| **合計** | **~3.8 MB** | **~15 MB** |

### メモリフットプリント（RSS）

| シナリオ | lwrcl（CycloneDDS） | ROS2 Humble（FastDDS） |
|--------|-------------------:|---------------------:|
| サブスクライバーノード起動直後 | **~15 MB** | ~38 MB |

lwrcl は実行時メモリ使用量が **約 2.5 倍少ない** です。

### エンドツーエンドレイテンシ（同一ホスト・プロセス内、C++ ノード）

| 指標 | lwrcl / CycloneDDS（`ddsperf -L`） | ROS2 Humble / FastDDS（rclcpp） |
|-----|----------------------------------:|--------------------------------:|
| 平均 | **~14 µs** | ~57 µs |
| p50 | **~13 µs** | ~32 µs |
| p90 | **~17 µs** | ~118 µs |
| p99 | **~20 µs** | ~375 µs |
| 最小 | **~5 µs** | ~20 µs |

lwrcl はパーセンタイル全体で **2〜7 倍低いレイテンシ** を達成しています。

> **注:** ROS2 の数値には rclcpp エグゼキューターのオーバーヘッド、`std_msgs::String` のシリアライゼーション、
> および rmw 抽象化レイヤーが含まれています。lwrcl の数値は `ddsperf` による CycloneDDS トランスポート層の
> 直接計測値であり、これは lwrcl がラップしている層そのものです。この比較は lwrcl ラッパーの薄さと、
> フルスタックの ROS2 が追加するオーバーヘッドの大きさを示しています。

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

Adaptive AUTOSAR バックエンドは ARXML 介在構成に対応しています。

- `build_apps.sh adaptive-autosar` 実行時に `autosar-generate-comm-manifest`（Adaptive-AUTOSAR 側でインストールされる PATH コマンド）がアプリソース内の `create_publisher` / `create_subscription` / `create_service` / `create_client` を走査し、mapping/manifest を自動生成します。
- 同時に `autosar-generate-proxy-skeleton`（Adaptive-AUTOSAR 側でインストールされる PATH コマンド）で mapping から proxy/skeleton ヘッダも自動生成します。
- 生成対象は `msg` topic だけでなく `srv` の request/response topic も含みます。
- 生成/配置される成果物:
  - `/opt/autosar-ap-libs/share/lwrcl/autosar/lwrcl_autosar_manifest.arxml`
  - `/opt/autosar-ap-libs/share/lwrcl/autosar/lwrcl_autosar_topic_mapping.yaml`
  - `/opt/autosar-ap-libs/share/lwrcl/autosar/lwrcl_autosar_manifest.yaml`
  - `/opt/autosar-ap-libs/share/lwrcl/autosar/lwrcl_autosar_manifest_dds.yaml`
  - `/opt/autosar-ap-libs/share/lwrcl/autosar/lwrcl_autosar_manifest_iceoryx.yaml`（codegen が `event_binding: iceoryx` に対応している場合）
  - `/opt/autosar-ap-libs/share/lwrcl/autosar/lwrcl_autosar_manifest_vsomeip.yaml`
- アプリビルド時に生成される proxy/skeleton ヘッダ:
  - `apps/build-adaptive-autosar/autosar/generated/lwrcl_autosar_proxy_skeleton.hpp`
- 実行時の `adaptive-autosar` Publisher/Subscription は生成済み `ara::com` Proxy/Skeleton を利用し、`ARA_COM_EVENT_BINDING`（推奨）でトランスポートを切り替えます。`ARA_COM_BINDING_MANIFEST` はフォールバックとして利用できます。

必要な Adaptive-AUTOSAR codegen コマンド:

- `autosar-generate-comm-manifest`
- `autosar-generate-proxy-skeleton`
- 既定のインストール先: `/opt/autosar_ap/bin`（Adaptive AUTOSAR ビルドスクリプトで PATH に追加）
- Adaptive-AUTOSAR プロジェクト側のスクリプト配置先: `tools/ara_com_codegen/`
- `autosar-generate-comm-manifest --help` に `iceoryx` が出ない場合は、Adaptive-AUTOSAR codegen ツールを更新するか `AUTOSAR_COMM_MANIFEST_GENERATOR` を上書きしてください。

Adaptive AUTOSAR mapping 関連の環境変数:

| 環境変数 | 用途 |
|---------|------|
| `ARA_COM_TOPIC_MAPPING` | 実行時 topic mapping YAML のパス上書き |
| `ARA_COM_REQUIRE_TOPIC_MAPPING=1` | mapping 未登録トピックをエラー扱い |
| `ARA_COM_DISABLE_TOPIC_MAPPING=1` | mapping を無効化して direct DDS 名を使用 |
| `ARA_COM_EVENT_BINDING` | 実行時トランスポート選択（`dds` / `iceoryx` / `vsomeip` / `auto`） |
| `ARA_COM_BINDING_MANIFEST` | `ARA_COM_EVENT_BINDING` 未指定時に使う実行時 fallback manifest パス |
| `AUTOSAR_APP_SOURCE_ROOT` | topic/service 抽出対象のアプリソースルート |
| `AUTOSAR_ARXML_GENERATOR` | ビルド時 ARXML 生成スクリプトのパス上書き |
| `AUTOSAR_COMM_MANIFEST_GENERATOR` | ビルド時 mapping 生成コマンドの上書き（既定: `autosar-generate-comm-manifest`） |
| `AUTOSAR_PROXY_SKELETON_GENERATOR` | ビルド時 proxy/skeleton 生成コマンドの上書き（既定: `autosar-generate-proxy-skeleton`） |
| `AUTOSAR_PROXY_SKELETON_PATCHER` | 生成済み proxy/skeleton ヘッダの後処理パッチスクリプト上書き（既定: `scripts/patch_autosar_proxy_skeleton_runtime_name.py`） |
| `AUTOSAR_EVENT_BINDING` | `lwrcl_autosar_manifest.yaml` の build 時既定 `event_binding`（既定: `auto`） |
| `AUTOSAR_GENERATE_BINDING_PROFILES=1` | `lwrcl_autosar_manifest_dds.yaml` / `lwrcl_autosar_manifest_iceoryx.yaml` / `lwrcl_autosar_manifest_vsomeip.yaml` も生成・インストール |
| `ARA_COM_ICEORYX_RUNTIME_NAME` | 実行時 iceoryx アプリ名の固定上書き（未指定時は PID 付きで自動生成して衝突回避） |
| `ARA_COM_ICEORYX_RUNTIME_PREFIX` | iceoryx アプリ名自動生成時の prefix（`<prefix>_<pid>`） |
| `VSOMEIP_PREFIX` | ビルド時 vsomeip インストール先の上書き（既定: `/opt/vsomeip`） |
| `VSOMEIP_CONFIGURATION` | 実行時 vsomeip 設定ファイルパス |

Adaptive AUTOSAR 実行時トランスポート切り替え（同一バイナリ、アプリコード変更不要）:

```bash
# CycloneDDS トランスポート profile
# （現行の Adaptive-AUTOSAR 参照実装では routing manager プロセスが必要）
unset ARA_COM_BINDING_MANIFEST
export ARA_COM_EVENT_BINDING=dds
export VSOMEIP_CONFIGURATION=/opt/autosar_ap/configuration/vsomeip-rpi.json
/opt/autosar_ap/bin/autosar_vsomeip_routing_manager &
apps/install-adaptive-autosar/bin/example_class_sub &
apps/install-adaptive-autosar/bin/example_class_pub

# iceoryx トランスポート profile
unset ARA_COM_BINDING_MANIFEST
export ARA_COM_EVENT_BINDING=iceoryx
iox-roudi &
apps/install-adaptive-autosar/bin/example_class_sub &
apps/install-adaptive-autosar/bin/example_class_pub

# SOME/IP トランスポート profile
unset ARA_COM_BINDING_MANIFEST
export ARA_COM_EVENT_BINDING=vsomeip
export VSOMEIP_CONFIGURATION=/opt/autosar_ap/configuration/vsomeip-rpi.json
/opt/autosar_ap/bin/autosar_vsomeip_routing_manager &
apps/install-adaptive-autosar/bin/example_class_sub &
apps/install-adaptive-autosar/bin/example_class_pub
```

両方の環境変数を設定した場合は、`ARA_COM_BINDING_MANIFEST` より `ARA_COM_EVENT_BINDING` が優先されます。

切り替え確認の観点:

- DDS profile: Subscriber が `I heard: 'Hello, world! ...'` を出力し、routing manager ログに `REGISTER EVENT` が出ないこと。
- iceoryx profile: `iox-roudi` 起動下で Subscriber が `I heard: 'Hello, world! ...'` を出力し、トランスポートエラーが出ないこと。
- SOME/IP profile: Subscriber が `I heard: 'Hello, world! ...'` を出力し、routing manager ログに `REGISTER EVENT` / `SUBSCRIBE` が出ること。

Adaptive AUTOSAR の Loaned Message API:

- `Publisher<T>::borrow_loaned_message()` と `publish(LoanedMessage<T>&&)` が利用できます。
- `Subscription<T>::take_loaned_message(LoanedSubscriptionMessage<T>&)` が利用できます。
- Adaptive AUTOSAR バックエンドの `can_loan_messages()` は `true` を返します（API は常に利用可能）。
- `ARA_COM_EVENT_BINDING=iceoryx` かつ `iox-roudi` 起動時、trivially-copyable な型は `ara::com::SkeletonEvent::Allocate/Send` 経路を利用できます。非 trivial な ROS メッセージ型は安全に typed `Send(const T&)` へフォールバックします。
- `adaptive-autosar` ビルドでも `example_zero_copy_pub` / `example_zero_copy_sub` を生成します。

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

iceoryx SHM 転送によるゼロコピーを有効化するには、以下を実行するだけです:

```bash
./scripts/install_cyclonedds.sh
```

`install_cyclonedds.sh` は iceoryx の有無を自動検出し、見つからない場合は `/opt/iceoryx` へのインストール（コンテナセーフ ACL フォールバック付き）を行ってから、`ENABLE_SHM=ON` で CycloneDDS をビルドします。

iceoryx の自動インストールをスキップして SHM なしでビルドするには:

```bash
./scripts/install_cyclonedds.sh --skip-iceoryx
# または
./scripts/install_cyclonedds.sh --disable-shm
```

インストール後、実行前に以下の環境変数を設定し `iox-roudi` を起動してください:

```bash
export LD_LIBRARY_PATH=/opt/iceoryx/lib:/opt/cyclonedds/lib:/opt/cyclonedds-libs/lib:${LD_LIBRARY_PATH}
export CYCLONEDDS_URI=file:///opt/cyclonedds/etc/cyclonedds-lwrcl.xml
iox-roudi
```

厳密な ACL 強制が必要な場合（デフォルトのコンテナセーフ動作を使わない場合）は、iceoryx を個別に再インストールしてください:

```bash
./scripts/install_iceoryx.sh --force --strict-acl
./scripts/install_cyclonedds.sh --skip-iceoryx --enable-shm
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
│   └── ...                     # Adaptive AUTOSAR codegen ツールは Adaptive-AUTOSAR プロジェクト側で提供
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
