[![FastDDS CI](https://github.com/tatsuyai713/lwrcl/actions/workflows/ci-fastdds.yml/badge.svg)](https://github.com/tatsuyai713/lwrcl/actions/workflows/ci-fastdds.yml)
[![CycloneDDS CI](https://github.com/tatsuyai713/lwrcl/actions/workflows/ci-cyclonedds.yml/badge.svg)](https://github.com/tatsuyai713/lwrcl/actions/workflows/ci-cyclonedds.yml)
[![vsomeip CI](https://github.com/tatsuyai713/lwrcl/actions/workflows/ci-vsomeip.yml/badge.svg)](https://github.com/tatsuyai713/lwrcl/actions/workflows/ci-vsomeip.yml)
[![Adaptive AUTOSAR CI](https://github.com/tatsuyai713/lwrcl/actions/workflows/ci-adaptive-autosar.yml/badge.svg)](https://github.com/tatsuyai713/lwrcl/actions/workflows/ci-adaptive-autosar.yml)

**[English (英語版)](README.md)**

# LWRCL (LightWeight Rclcpp Compatible Library)

**ROS 2 の rclcpp に似た API を持つ、軽量な DDS 通信ライブラリ**

lwrcl は、ROS 2 の rclcpp と互換性のある API を提供する軽量ライブラリです。通信バックエンドとして **CycloneDDS**、**FastDDS**、**RTI Connext DDS**、**vsomeip (SOME/IP)**、**Adaptive AUTOSAR (ara::com)** に対応しています。ネイティブ DDS バックエンドは ROS 2 をフルインストールせず、互換性のある ROS 2 DDS エンドポイントと直接通信できます。SOME/IP と ara::com は独自の wire protocol と service model を使用します。

---

## 特徴

- **rclcpp 互換 API** — `rclcpp` 名前空間を使用しているため、既存の ROS 2 コードを少ない変更で移植できます。`RCLCPP_INFO` / `RCLCPP_WARN` などのログマクロも利用可能です。
- **ROS 2 との相互通信** — ネイティブDDSバックエンドは、同一Domain上の互換ROS 2 DDSエンドポイントと直接通信できます。SOME/IPとara::comでは適切なmappingまたはbridgeが必要です。
- **バックエンド選択** — CycloneDDS、FastDDS、RTI Connext DDS、vsomeip、Adaptive AUTOSAR をビルド時に選択できます。複数同時にインストールしておき、用途に応じて切り替えることも可能です。
- **依存関係が少ない** — DDS ライブラリと yaml-cpp のみに依存しており、ビルドが高速です。
- **マルチプラットフォーム** — Linux (Ubuntu/Debian)、macOS (Homebrew)、QNX 8.0 に対応しています。
- **tf2 / tf2_ros 同梱** — 座標変換の基本機能を同梱しています。

---

## 対応する通信バックエンド

| バックエンド | 特徴 |
|----------|------|
| **CycloneDDS** | Eclipse Foundation のオープンソース実装。軽量。 |
| **FastDDS** | eProsima 製。QoS オプションが豊富。 |
| **RTI Connext DDS** | RTI Modern C++ API と生成 TypePlugin を使用する商用 DDS 実装。 |
| **vsomeip** | COVESA の SOME/IP 実装。DDS ランタイム不要の車載グレード通信。 |
| **Adaptive AUTOSAR** | `ara::com` ベースのバックエンド（Adaptive-AUTOSAR 連携）。 |

---

## バックエンドの実装・仕様の違い

各バックエンドは同じ lwrcl／rclcpp 互換 API を公開しますが、wire protocol
レベルで相互に置き換えられるわけではありません。FastDDS、CycloneDDS、RTI
Connext DDS はネイティブ DDS 実装です。vsomeip は API を SOME/IP に変換し、
Adaptive AUTOSAR は生成された `ara::com` proxy/skeleton event に変換します。
したがってバックエンドを変更すると、リンクするライブラリだけでなく、Discovery、
serialization、QoS の適用範囲、deployment artifact、zero-copy の条件も変わります。

### アーキテクチャ比較

| 項目 | FastDDS | CycloneDDS | RTI Connext DDS | vsomeip | Adaptive AUTOSAR |
|---|---|---|---|---|---|
| Transport／通信モデル | DDS/RTPS | DDS/RTPS | DDS/RTPS | SOME/IP event／method | `ara::com` service／event |
| lwrcl が使用するネイティブ C++ API | eProsima Fast DDS API | OMG DDS C++ (`dds::...`) | RTI Modern C++ (`dds::...`, `rti::...`) | COVESA `vsomeip` | AUTOSAR `ara::com` proxy/skeleton API |
| 中心となるruntime entity | `DomainParticipant` | `dds::domain::DomainParticipant` | `dds::domain::DomainParticipant` | `vsomeip::application` | `ara::com` runtime と生成proxy/skeleton |
| Topic の実装 | DDS Topic/DataWriter/DataReader | DDS Topic/DataWriter/DataReader | DDS Topic/DataWriter/DataReader | hash化したservice ID＋event/eventgroup | 生成service instance＋event |
| Service の実装 | request/response用DDS Topicの組 | request/response用DDS Topicの組 | request/response用DDS Topicの組 | SOME/IP request/response method | 互換層が生成proxy/skeleton eventをrequest/responseとして使用 |
| Discovery | DDS participant/endpoint discovery | DDS participant/endpoint discovery | DDS participant/endpoint discovery | SOME/IP-SD | `ara::com` service discovery。下位処理はbinding依存 |
| ROS 2 DDS wire互換 | Topic名・型名・QoSが互換なら対応 | Topic名・型名・QoSが互換なら対応 | Topic名・型名・QoSが互換なら対応 | 非対応。bridge/gatewayが必要 | 自動では非対応。互換mappingまたはbridgeが必要 |
| Runtime license | オープンソース | オープンソース | 導入したRTI bundleによってライセンスが必要 | オープンソース | AUTOSAR platform/runtime distributionに依存 |

ソース構造が最も近いのは CycloneDDS と RTI DDS です。どちらも
`dds::domain`、`dds::pub`、`dds::sub`、`dds::topic` というOMG形式のAPIを
使用します。FastDDSは同じDDSの概念をeProsima固有APIで実装します。
vsomeipとAdaptive AUTOSARはDDS実装の差し替えではなく、異なる通信モデルへ
変換するprotocol adapterです。

### 型生成とSerialization

| バックエンド | 型／コード生成 | 生成されるサポート | Serialization経路 |
|---|---|---|---|
| FastDDS | `fastddsgen -typeros2` | Fast DDS TopicDataType/TypeSupport | 生成Fast DDS型サポート経由のFast CDR |
| CycloneDDS | `idlc -l cxx` | CycloneDDS C++ topic traits | CycloneDDS CDR実装 |
| RTI Connext DDS | `rtiddsgen -language C++11 -unboundedSupport` | RTI TypeSupport／TypePlugin | `dds::topic::topic_type_support<T>::to_cdr_buffer()`／`from_cdr_buffer()` |
| vsomeip | CycloneDDS生成のROS型定義 | DDS runtimeへの型登録なし | standalone CycloneDDS CDRでSOME/IP payloadへserialize |
| Adaptive AUTOSAR | CycloneDDS ROS型＋lwrcl AUTOSAR mapping生成 | `TopicEventProxy<T>`／`TopicEventSkeleton<T>`とmanifest | 選択した`ara::com` binding/runtimeが処理 |

生成型はすべて、lwrclが期待するROS形式のソースAPIへ正規化されます。具体的には
public field、`SharedPtr`／`ConstSharedPtr`、snake_case include wrapper、serviceの
Request/Response wrapperを生成します。RTIではstock `rtiddsgen`がaccessor methodを
生成し、非境界IDL sequenceにもデフォルト上限100を設定するため、追加のpatch処理が
必要です。RTI型リポジトリでは`-unboundedSupport`を有効にし、
`sensor_msgs::msg::Image::data`のようなROS arrayを実際の非境界sequenceとして
生成します。

生成型はバックエンド固有のABI artifactです。同一application binary内で
`/opt/fast-dds-libs`、`/opt/cyclonedds-libs`、`/opt/rtidds-libs`、
`/opt/vsomeip-libs`、`/opt/autosar-ap-libs`のheaderやlibraryを混在させないで
ください。

### QoSの違い

`rclcpp::QoS` APIは全バックエンドにありますが、transportへ実際に適用される範囲が
異なります。

| QoS | FastDDS | CycloneDDS | RTI Connext DDS | vsomeip | Adaptive AUTOSAR |
|---|---|---|---|---|---|
| History／depth | DDSへ反映 | DDSへ反映 | DDSへ反映 | depthはlwrclローカル受信bufferの上限として使用 | depthはlwrcl受信bufferへ反映。transportはbinding依存 |
| Reliability | DDSへ反映 | DDSへ反映 | DDSへ反映 | SOME/IP transport QoSへは未変換 | binding／deployment定義に依存 |
| Durability | DDSへ反映 | DDSへ反映 | DDSへ反映 | 未変換 | binding／deployment定義に依存 |
| Deadline／lifespan／liveliness | FastDDSバックエンドがDDSへ反映 | APIでは保持するが、現在のPublisher/Subscriptionはhistory、reliability、durabilityを中心に反映 | APIでは保持するが、現在のPublisher/Subscriptionはhistory、reliability、durabilityを中心に反映 | API互換値のみ | deployment／binding定義に依存 |
| 大容量Reliable sample | DDS fragmentation | DDS fragmentation | Reliable fragmentationではRTI asynchronous publish modeを使用 | SOME/IP payload／routing上限に依存 | binding固有の上限に依存 |

RTI Modern C++のQoS objectは、所有entityの設定済みdefaultから初期化する必要が
あります。そのためRTIバックエンドは`participant.default_topic_qos()`、
`publisher.default_datawriter_qos()`、`subscriber.default_datareader_qos()`を
取得してからlwrclのoverrideを適用します。またRTIは、同一Participant内で同じ名前・
同じ型のTopicを再assertすると拒否するため、既存のtyped Topicを検索して再利用します。

### Loaned MessageとZero Copy

application codeを共通化するため、すべてのバックエンドに
`borrow_loaned_message()`と`take_loaned_message()`があります。ただし、
`can_loan_messages()`はAPIが利用可能であることを表し、end-to-endで必ずzero copyに
なることを保証しません。

| バックエンド | Publisher側 | Subscriber側 | 条件／制約 |
|---|---|---|---|
| FastDDS | native `DataWriter::loan_sample()`。失敗時はheap fallback | native DataReader loan／return | Fast DDS transport、memory policy、型によって利用可否が変わる |
| CycloneDDS | native writer loan。失敗時はheap fallback | `dds::sub::LoanedSamples<T>` | native SHM zero copyにはCycloneDDS SHM、iceoryx、互換性のある固定長型が必要 |
| RTI Connext DDS | 固定長の生成ROS型はnative `DataWriter::get_loan()`、失敗時はheap fallback | RTI `dds::sub::LoanedSamples<T>` | 固定長IDLは`SHMEM_REF`として生成。string／sequenceはfallbackで通常のSHMまたはnetwork transportを使用 |
| vsomeip | heap messageをCDR serialize後、SOME/IP payloadへ格納 | payloadを中間copyなしで参照した後、ROS objectへdeserialize | loan API互換であり、end-to-end zero copyではない |
| Adaptive AUTOSAR | binding対応時は`SkeletonEvent::Allocate()`、非対応時はheap fallback | `ara::com::SamplePtr<T>` | zero copy可否は選択したbinding依存。mutable access時はcopyを生成 |

RTI用の型generatorは、各ROS IDLとinclude先を再帰的に判定し、object graph全体が
固定長の場合だけ`@transfer_mode(SHMEM_REF)`を付与します。
`borrow_loaned_message()`はRTIの`DataWriter::get_loan()`から領域を取得し、未publishの
loanは`discard_loan()`で返却します。`write()`に成功した場合は所有権がRTIへ戻ります。
型に`string`、`wstring`、`sequence`が含まれる場合、またはRTIのloan poolが一時的に
枯渇した場合だけ、portable messageをheapに確保します。

RTI shared-memory transportとRTI zero-copy transferは同義ではありません。通常の可変長
ROS型もshared-memory transportで送れますが、STL表現をserializeし、transport bufferへ
copyします。RTI FlatDataのmutable型なら可変長データをloan済みserialized buffer内で
構築できますが、Builder／Offset APIを使う別のC++型であり、patch済みROS型の
`std::string`／`std::vector` APIとは一致しません。そのため、このバックエンドは公開
ROS message型を暗黙にFlatDataへ置き換えません。

### 設定・Deploymentの違い

| バックエンド | 主なruntime設定 | デフォルトinstall先 | 主な要件 |
|---|---|---|---|
| FastDDS | `FASTDDS_DEFAULT_PROFILES_FILE`とFast DDS XML | `/opt/fast-dds-libs` | Fast DDS、Fast CDR、FastDDS用生成ROS型 |
| CycloneDDS | `CYCLONEDDS_URI`とCycloneDDS XML | `/opt/cyclonedds-libs` | CycloneDDS/CycloneDDS-CXX。SHMにはiceoryxとRouDiが必要 |
| RTI Connext DDS | RTI QoS profile、`NDDSHOME`、`CONNEXTDDS_ARCH`、必要に応じて`RTI_LICENSE_FILE` | `/opt/rtidds-libs` | 対象architectureの`rtisetenv`をsourceし、導入bundle／機能に合うlicenseを使用 |
| vsomeip | vsomeip JSON、routing manager、application/service設定 | `/opt/vsomeip-libs` | runtimeにBoostとvsomeip。CycloneDDS `idlc`／CDR部分はbuild-time serialization依存でありDDS transport依存ではない |
| Adaptive AUTOSAR | ARXMLと生成mapping／manifest。`ARA_COM_EVENT_BINDING`で`vsomeip`、`dds`、`iceoryx`を選択 | `/opt/autosar-ap-libs` | Adaptive AUTOSAR AP runtime、生成proxy/skeleton metadata、選択bindingの依存関係 |

現在のproject scriptは、FastDDS／CycloneDDSではLinux・macOS・QNX、vsomeipでは
依存関係を導入できるLinux・macOS・QNX、Adaptive AUTOSARではLinux・QNXを対象に
しています。RTIバックエンドは現在RTI Connext DDS 7.3.0＋x86_64 Linuxでbuild／test
済みです。他のRTI targetでは、一致するlicensed architecture packageが必要で、
platform固有build scriptの追加が必要になる場合があります。

RTI licenseが複数存在する場合は`RTI_LICENSE_FILE`を明示してください。明示しない場合、
RTI applicationはcurrent directoryと`NDDSHOME`も探索するため、意図したproduction
licenseとは異なるファイルを選択する可能性があります。

Adaptive AUTOSARは他のバックエンドより多くのbuild-time deployment metadataを
必要とします。`build_apps.sh adaptive-autosar`はpublisher、subscription、service、
clientの生成箇所をscanし、applicationが使うmapping、manifest、proxy/skeleton headerを
生成します。TopicやServiceを変更した場合は、これらartifactの再生成と再deploymentが
必要になることがあります。

### バックエンド選択の目安

- **CycloneDDS**: 小さいopen-source DDS runtime、ROS 2 wire互換、iceoryx SHMを
  利用したい場合。
- **FastDDS**: Fast DDS ecosystemとの互換、拡張QoS mapping、native loan APIを
  重視する場合。
- **RTI Connext DDS**: RTIを標準採用している環境、商用tool/support、RTI製品間の
  interoperabilityが必要で、対象RTI packageとlicenseを利用できる場合。
- **vsomeip**: DDS DiscoveryやDDS wire互換を必要とせず、native SOME/IPとして
  deploymentする場合。
- **Adaptive AUTOSAR**: 公開communication boundaryを`ara::com`とし、AUTOSAR
  manifestと生成proxy/skeleton artifactでdeploymentを管理する場合。

---

## 機能対応表

| 機能 | lwrcl | rclcpp |
|------|:-----:|:------:|
| Node | ✅ | ✅ |
| Publisher / Subscription | ✅ | ✅ |
| Service / Client | ✅ | ✅ |
| Timer (create_timer / create_wall_timer) | ✅ | ✅ |
| Executor (Single / Multi-threaded) | ✅ | ✅ |
| QoS (Reliability, Durability, History) | ✅（mappingはバックエンド依存） | ✅ |
| QoS (Deadline, Lifespan, Liveliness) | ⚠️ バックエンド依存 | ✅ |
| QoS プリセット (SensorDataQoS 等) | ✅ | ✅ |
| Parameter (declare, get, set) | ✅ | ✅ |
| Parameter File (YAML) | ✅ | ✅ |
| Zero Copy (Loaned Messages) | ⚠️ バックエンド／型／transport依存 | ✅ |
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

以下の数値はすべて **x86\_64 Linux ホスト**（Intel Core i9-14900K、Ubuntu 24.04、kernel 6.8）上で計測しました。
レイテンシテストは C++ プロセス内ピンポンノード（`std_msgs::msg::String`、1 ms タイマー、10 000 サンプル、2 000 ウォームアップ）を使用しています。
数値は**往復遅延 ÷ 2**（片道）で、各値は 5 回の独立した計測の中央値です。

### 1. ライブラリ・バイナリサイズ

| コンポーネント | ファイル | サイズ（デバッグ情報付き） | サイズ（ストリップ後） |
|-------------|--------|-------------------:|----------------:|
| lwrcl ラッパー | `liblwrcl.so` (CycloneDDS) | 3.1 MB | **793 KB** |
| lwrcl ラッパー | `liblwrcl.so` (FastDDS) | 3.5 MB | **928 KB** |
| lwrcl ラッパー | `liblwrcl.so` (vsomeip) | 2.5 MB | **683 KB** |
| CycloneDDS ランタイム | `libddsc.so` | — | 1.6 MB |
| CycloneDDS C++ バインディング | `libddscxx.so` | — | 1.4 MB |
| FastDDS ランタイム | `libfastrtps.so` | — | 12 MB |
| アプリケーションバイナリ | `example_class_pub` (CycloneDDS) | 208 KB | **143 KB** |
| アプリケーションバイナリ | `example_class_pub` (vsomeip) | 943 KB | 143 KB |

> lwrcl ラッパー本体（`liblwrcl.so`）はストリップ後わずか **約 700〜930 KB** — DDS ランタイムの薄いラッパー層です。
> パブリッシャー/サブスクライバーアプリケーションのバイナリは **ストリップ後 150 KB 未満** です。

### 2. 実行時メモリフットプリント

起動直後の単一サブスクライバーノード（`example_class_sub`、CycloneDDS バックエンド）で計測：

| 指標 | 値 |
|------|---:|
| VmRSS（常駐セットサイズ） | **~15 MB** |
| VmSize（仮想メモリ） | ~635 MB\* |

\* 仮想メモリが大きいのは DDS 共有メモリ領域の予約によるもので、実際の物理メモリ使用量は RSS の値が示します。

### 3. エンドツーエンドレイテンシ（コールバック API、6 パターン比較）

C++ プロセス内ピンポンノード（`std_msgs::msg::String`、1 ms タイマー、10 000 サンプル、2 000 ウォームアップ）で計測。
数値は**往復遅延 ÷ 2**（片道レイテンシ）で、各値は同一ホスト上での 5 回計測の中央値です。

| 指標   | lwrcl + CycloneDDS | lwrcl + FastDDS | lwrcl + vsomeip | lwrcl + Adaptive AUTOSAR (vsomeip) | lwrcl + Adaptive AUTOSAR (DDS) | lwrcl + Adaptive AUTOSAR (iceoryx) | ROS 2 Jazzy + CycloneDDS | ROS 2 Jazzy + FastDDS |
|-------|-------------------:|----------------:|----------------:|----------------------------------:|-------------------------------:|-----------------------------------:|------------------------:|---------------------:|
| p50   | **~26 µs**         | ~33 µs          | ~65 µs          | ~206 µs                           | ~81 µs                         | ~43 µs                             | ~17 µs                  | ~15 µs               |
| p90   | **~30 µs**         | ~47 µs          | ~111 µs         | ~273 µs                           | ~133 µs                        | ~79 µs                             | ~38 µs                  | ~61 µs               |
| p99   | ~45 µs             | ~92 µs          | ~198 µs         | ~433 µs                           | ~222 µs                        | ~170 µs                            | **~43 µs**              | ~62 µs               |
| 最小   | ~2 µs              | ~2 µs           | ~10 µs          | ~27 µs                            | ~11 µs                         | ~7 µs                              | ~2 µs                   | ~3 µs                |

> ノード内の全サブスクリプションは単一の**統合ノードレベル WaitSet** を共有します。
> 全 `ReadCondition` を一つの WaitSet に集約することで、スピンスレッドはメッセージ到着ごとに
> 1 回だけ起床します。コールバックはスピンスレッドから直接呼び出します（エグゼキューター経由なし）。
> ノードレベルのミューテックスで並行コールバックをシリアライズします。
>
> lwrcl + CycloneDDS は p50 で **~26 µs の片道レイテンシ** を達成し、ROS 2 Jazzy と同等の性能です。
>
> Adaptive AUTOSAR のトランスポートバインディングは `ARA_COM_EVENT_BINDING`（vsomeip | dds | iceoryx）で選択可能です。
> iceoryx バインディングは p50 で **~43 µs** を達成し、3 つの Adaptive AUTOSAR トランスポートの中で最速です。

**生 CycloneDDS トランスポート層**（`ddsperf -L ping pong`、lwrcl コールバック層をバイパス）：

| メッセージサイズ | p50 | p90 | p99 | 最小 |
|--------------|----:|----:|----:|----:|
| 12 バイト | 13.0 µs | 17.1 µs | 21 µs | 5 µs |
| 32 バイト | 12.8 µs | 16.7 µs | 19 µs | 4 µs |
| 256 バイト | 13.2 µs | 17.3 µs | 19 µs | 1 µs |

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

| バックエンド | 合計（i9-14900K、32 スレッド） |
|-----------|------:|
| CycloneDDS | **~6 秒** |
| FastDDS | **~6 秒** |
| vsomeip | **~4 秒** |

> lwrcl ライブラリ全体のビルドは、最新のマシンで **10 秒未満**で完了します。

### 6. ソースコード量

バックエンドごとの C++ ソースコード総行数（`.cpp` ＋ `.hpp`）：

| バックエンド | 行数（ヘッダー＋ソース） |
|-----------|---------------------:|
| CycloneDDS | ~5,100 |
| FastDDS | ~5,300 |
| vsomeip | ~4,400 |
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

## ROS 2 Jazzy との比較

すべての計測は同一マシン（Intel Core i9-14900K、x86\_64、Ubuntu 24.04）上で行いました。
レイテンシは C++ プロセス内ピンポンノード（`std_msgs::msg::String`、10 000 サンプル、2 000 ウォームアップ）で計測し、
数値は**往復遅延 ÷ 2**（片道）・5 回計測の**中央値**です。

### ミドルウェア層のサイズ

| コンポーネント | lwrcl + CycloneDDS | lwrcl + FastDDS | lwrcl + vsomeip | lwrcl + Adaptive AUTOSAR | ROS 2 + CycloneDDS | ROS 2 + FastDDS |
|-------------|-------------------:|----------------:|----------------:|------------------------:|------------------:|---------------:|
| ラッパー / rclcpp 層 | **793 KB** | **928 KB** | **683 KB** | — | 3.0 MB | 3.0 MB |
| DDS ランタイム | 3.0 MB | 12 MB | — | — | 3.0 MB | 12 MB |
| **合計** | **~3.8 MB** | **~13 MB** | **~0.7 MB** | — | **~6.0 MB** | **~15 MB** |

### エンドツーエンドレイテンシ（同一ホスト・プロセス内、C++ コールバック API）

| 指標   | lwrcl + CycloneDDS | lwrcl + FastDDS | lwrcl + vsomeip | lwrcl + Adaptive AUTOSAR (vsomeip) | lwrcl + Adaptive AUTOSAR (DDS) | lwrcl + Adaptive AUTOSAR (iceoryx) | ROS 2 Jazzy + CycloneDDS | ROS 2 Jazzy + FastDDS |
|-------|-------------------:|----------------:|----------------:|----------------------------------:|-------------------------------:|-----------------------------------:|------------------------:|---------------------:|
| p50   | **~26 µs**         | ~33 µs          | ~65 µs          | ~206 µs                           | ~81 µs                         | ~43 µs                             | ~17 µs                  | ~15 µs               |
| p90   | **~30 µs**         | ~47 µs          | ~111 µs         | ~273 µs                           | ~133 µs                        | ~79 µs                             | ~38 µs                  | ~61 µs               |
| p99   | ~45 µs             | ~92 µs          | ~198 µs         | ~433 µs                           | ~222 µs                        | ~170 µs                            | **~43 µs**              | ~62 µs               |
| 最小   | ~2 µs              | ~2 µs           | ~10 µs          | ~27 µs                            | ~11 µs                         | ~7 µs                              | ~2 µs                   | ~3 µs                |

> lwrcl は統合ノードレベル WaitSet を使用：全サブスクリプションの `ReadCondition` を集約し、
> `Node::spin()` は単一の WaitSet でブロックして 1 メッセージ到着につき 1 回だけ起床します。
> コールバックはスピンスレッドから直接呼び出します（エグゼキューター経由なし）。
> ノードレベルのミューテックスで並行コールバックをシリアライズします。

### まとめ

| 指標 | lwrcl + CycloneDDS | lwrcl + FastDDS | lwrcl + vsomeip | lwrcl + Adaptive AUTOSAR (iceoryx) | ROS 2 + CycloneDDS | ROS 2 + FastDDS |
|-----|:------------------:|:---------------:|:---------------:|:----------------------------------:|:-----------------:|:--------------:|
| ミドルウェアサイズ | **~3.8 MB** ✅ | ~13 MB | **~0.7 MB** ✅ | — | ~6.0 MB | ~15 MB |
| レイテンシ p50 | ~26 µs | ~33 µs | ~65 µs | **~43 µs** | ~17 µs | ~15 µs |
| レイテンシ p90 | **~30 µs** | ~47 µs | ~111 µs | ~79 µs | ~38 µs | ~61 µs |
| レイテンシ p99 | **~45 µs** | ~92 µs | ~198 µs | ~170 µs | ~43 µs | ~62 µs |
| 移植性 | **QNX / AUTOSAR** ✅ | **QNX** ✅ | **QNX / AUTOSAR** ✅ | **AUTOSAR AP** ✅ | Linux のみ | Linux のみ |
| ROS 2 不要 | **Yes** ✅ | **Yes** ✅ | **Yes** ✅ | **Yes** ✅ | No | No |

---

## 依存関係

- CMake 3.16.3 以上
- C++14 対応コンパイラ
- 通信バックエンド（CycloneDDS、FastDDS、RTI Connext DDS、vsomeip、または Adaptive AUTOSAR）
- yaml-cpp（サブモジュールとして同梱）
- Boost（vsomeip バックエンド使用時に必要）
- macOS では Homebrew を使用します。互換性のため、`install_cmake_macos.sh` で CMake 3.31.6、`install_boost_macos.sh` で Boost 1.85 を導入します。

---

## ビルド手順 (Linux)

すべてのビルドスクリプトは以下の形式で実行します:

```
./build_<target>.sh <fastdds|cyclonedds|rtidds|vsomeip|adaptive-autosar> [install|clean]
```

- 第 1 引数: 通信バックエンド（`fastdds`、`cyclonedds`、`rtidds`、`vsomeip`、または `adaptive-autosar`）
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

**RTI Connext DDS:**

RTI Connext DDS本体を別途導入し、対象architectureの環境scriptをsourceして、
production licenseを明示してからビルドします。

```bash
source "$HOME/rti_connext_dds-7.3.0/resource/scripts/rtisetenv_x64Linux4gcc7.3.0.bash"
export RTI_LICENSE_FILE=/path/to/production/rti_license.dat
./build_all.sh rtidds install
```

`rtidds`ビルドはlwrcl、生成ROS型library、yaml-cppを`/opt/rtidds-libs`へ
インストールします。RTI SDK本体やlicenseは再配布しません。

**vsomeip (SOME/IP):**

```bash
./scripts/install_vsomeip.sh
```

> vsomeip は Boost と CycloneDDS（ビルド時の `idlc` コード生成ツール用のみ）が必要です。先に CycloneDDS をインストールしてから `install_vsomeip.sh` を実行してください。vsomeip バックエンドは DDS ランタイムに**依存しません** — cyclonedds-cxx から抽出した CDR シリアライゼーションをスタンドアロンの静的ライブラリとして使用します。

**Adaptive AUTOSAR (ara::com):**

- Adaptive AUTOSAR AP ランタイムを `/opt/autosar-ap` に導入します（例: [`Adaptive-AUTOSAR`](https://github.com/tatsuyai713/Adaptive-AUTOSAR)）。
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
> **RTI Connext DDSを使う場合は、`rtisetenv`をsourceし、`fastdds`を`rtidds`に置き換えてください。**
> **vsomeip を使う場合は、上記コマンドの `fastdds` を `vsomeip` に置き換えてください。**
> **Adaptive AUTOSAR の場合は、以下の専用手順を実行してください。**

## ビルド手順 (macOS / Homebrew)

macOS では Homebrew でビルドツールを導入し、DDS ランタイムと lwrcl は Linux と同じ `/opt` 配下へインストールします。Apple Silicon / Intel Mac のどちらでも Homebrew prefix は自動検出します。

```bash
brew install git openssl@3 automake autoconf libtool bison flex
./scripts/install_boost_macos.sh
./scripts/install_cmake_macos.sh
```

`install_boost_macos.sh` は vsomeip 互換性のために Boost 1.85 をローカル Homebrew tap から導入します。`install_cmake_macos.sh` は Homebrew の CMake を 3.31.6 に下げて pin します。これにより、古い Fast-DDS 依存ライブラリで発生する CMake 4 の policy エラーを回避します。

CycloneDDS バックエンドの最短手順:

```bash
./scripts/install_cyclonedds.sh
./build_all.sh cyclonedds install
```

インストール先:

| バックエンド | DDS ランタイム | lwrcl / データ型 |
|-------------|----------------|------------------|
| CycloneDDS | `/opt/cyclonedds` | `/opt/cyclonedds-libs` |
| FastDDS | `/opt/fast-dds` | `/opt/fast-dds-libs` |
| vsomeip | `/opt/vsomeip` | `/opt/vsomeip-libs` |

FastDDS バックエンドを使う場合:

```bash
./scripts/install_fast_dds.sh
./scripts/install_fast_dds_gen.sh
./build_all.sh fastdds install
```

vsomeip バックエンドを使う場合:

```bash
./scripts/install_cyclonedds.sh
./scripts/install_vsomeip.sh
./build_all.sh vsomeip install
```

実行時に dylib が見つからない場合は、使用するバックエンドに応じて以下を設定してください:

```bash
export DYLD_LIBRARY_PATH=/opt/iceoryx/lib:/opt/cyclonedds/lib:/opt/cyclonedds-libs/lib:${DYLD_LIBRARY_PATH:-}
```

macOS でも標準で iceoryx 共有メモリ transport を有効化します。`install_cyclonedds.sh` は `/opt/iceoryx` を自動検出し、見つからない場合は `install_iceoryx.sh` でインストールしてから `ENABLE_SHM=ON` で CycloneDDS をビルドします。SHM なしでビルドしたい場合のみ `./scripts/install_cyclonedds.sh --disable-shm` または `--skip-iceoryx` を指定してください。

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
- 既定のインストール先: `/opt/autosar-ap/bin`（Adaptive AUTOSAR ビルドスクリプトで PATH に追加）
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
export VSOMEIP_CONFIGURATION=/opt/autosar-ap/configuration/vsomeip-rpi.json
/opt/autosar-ap/bin/autosar_vsomeip_routing_manager &
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
export VSOMEIP_CONFIGURATION=/opt/autosar-ap/configuration/vsomeip-rpi.json
/opt/autosar-ap/bin/autosar_vsomeip_routing_manager &
apps/install-adaptive-autosar/bin/example_class_sub &
apps/install-adaptive-autosar/bin/example_class_pub
```

両方の環境変数を設定した場合は、`ARA_COM_BINDING_MANIFEST` より `ARA_COM_EVENT_BINDING` が優先されます。

切り替え確認の観点:

- DDS profile: Subscriber が `I heard: 'Hello, world! ...'` を出力し、routing manager ログに `REGISTER EVENT` が出ないこと。
- iceoryx profile: `iox-roudi` 起動下で Subscriber が `I heard: 'Hello, world! ...'` を出力し、トランスポートエラーが出ないこと。
- SOME/IP profile: Subscriber が `I heard: 'Hello, world! ...'` を出力し、routing manager ログに `REGISTER EVENT` / `SUBSCRIBE` が出ること。

Adaptive AUTOSAR の transport 高速化:

- ユーザーコードは同じ `Publisher<T>::publish(const T&)` と subscription callback API を使います。
- `ARA_COM_EVENT_BINDING=iceoryx` かつ `iox-roudi` 起動時、trivially-copyable な型は内部的に iceoryx-backed 経路を利用できます。非 trivial な ROS メッセージ型は安全に typed `Send(const T&)` へフォールバックします。
- `adaptive-autosar` ビルドでも `example_zero_copy_pub` / `example_zero_copy_sub` は通常 publish/subscribe の例として生成します。

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
| Adaptive AUTOSAR | `/opt/autosar-ap` | `/opt/autosar-ap-libs`（`build_libraries.sh adaptive-autosar` で `yaml-cpp` も導入） |

---

## CycloneDDS の Zero Copy (iceoryx)

CycloneDDS バックエンドでは、lwrcl は利用可能な場合に native の writer-loan/read-loan API を使用し、利用できない場合は安全にフォールバックします。native loan を厳密に確認する場合は、可変長 sequence を含む通常の ROS メッセージではなく、`fixedsized_msgs` のような固定長メッセージ型を使用してください。

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

lwrcl の install 時には、既定のランタイムプロファイルも自動でインストールされます:

- Fast DDS: `/opt/fast-dds-libs/etc/fastdds.xml`
- CycloneDDS: `/opt/cyclonedds-libs/etc/cyclonedds-lwrcl.xml`

環境変数で明示指定されていない場合、lwrcl はこれらのプロファイルを自動で読み込みます。Fast DDS は SHM と UDP の両方を有効にしているため、同一ホスト内では SHM/DataSharing を使い、別ホスト相手では ROS 2 風 API を変えずに UDP へ fallback できます。

CycloneDDS SHM を使う場合は、実行前に `iox-roudi` を起動してください:

```bash
export LD_LIBRARY_PATH=/opt/iceoryx/lib:/opt/cyclonedds/lib:/opt/cyclonedds-libs/lib:${LD_LIBRARY_PATH}
export DYLD_LIBRARY_PATH=/opt/iceoryx/lib:/opt/cyclonedds/lib:/opt/cyclonedds-libs/lib:${DYLD_LIBRARY_PATH:-}
iox-roudi
```

明示的にプロファイルを上書きしたい場合は、従来通り環境変数を指定できます:

```bash
export CYCLONEDDS_URI=file:///opt/cyclonedds-libs/etc/cyclonedds-lwrcl.xml
export FASTDDS_DEFAULT_PROFILES_FILE=/opt/fast-dds-libs/etc/fastdds.xml
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

高速化 transport の確認には、バックエンド別テストランナーを使用してください。`test_shm_zero_copy` は `fixedsized_msgs::msg::FixedImageVgaMono8` を使い、通常の ROS 2 風 `Publisher::publish(const T&)` と subscription callback API のまま動作することを確認します。アプリ側でバックエンド固有の message 型や loaned-message API に切り替える必要はありません。このテストでは有効なバックエンドプロファイルも確認するため、UDP のみ、または localhost 固定だけの設定はゼロコピー/SHM 設定として通りません。

```bash
./test/run_tests_fastdds.sh all
./test/run_tests_cyclonedds.sh all
```

SHM/DataSharing は通常 API の既定高速経路として設定され、テストも通常の publish/subscribe API で実行します。

---

## テスト

バックエンド別の gtest は `test/` 配下のランナーで実行できます。

```bash
./test/run_tests_fastdds.sh all
./test/run_tests_cyclonedds.sh all
./test/run_tests_vsomeip.sh all
```

ローカル smoke test はバックエンドを指定できます。

```bash
./run_smoke_local.sh --backend cyclonedds
./run_smoke_local.sh --backend fastdds
./run_smoke_local.sh --backend vsomeip
```

ビルド済み成果物を使って実行だけ確認する場合は `--no-build` を指定します。

```bash
./run_smoke_local.sh --backend cyclonedds --no-build
```

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
| Adaptive AUTOSAR | AUTOSAR AP: `/opt/qnx/autosar-ap/aarch64le`、CycloneDDS: `/opt/qnx/cyclonedds` | `/opt/qnx/autosar-ap-libs`（`build_libraries_qnx.sh adaptive-autosar` で `yaml-cpp` も導入） |

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
