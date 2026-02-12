# Advanced Examples

This directory contains advanced lwrcl examples that demonstrate real-world usage patterns with multiple features composed together.

## Examples

### 1. Video Stream Pipeline (`video_stream_pipeline`)

Simulates an H265 video encoding pipeline with two interconnected nodes.

**Features demonstrated:**
- Custom IDL message types (`VideoStreamStatus`)
- Parameter-driven configuration (`--params-file`)
- Shared `DomainParticipant` across multiple nodes
- `SingleThreadedExecutor` with multiple nodes
- Multiple QoS profiles (`SensorDataQoS` for video, `ReliableQoS` for status)
- Wall timers with different periods
- Namespace-aware topic routing (`/camera/...`)
- Structured logging

**Run:**
```bash
./video_stream_pipeline --params-file config/video_stream_pipeline.yaml
```

---

### 2. Diagnostic Aggregator (`diagnostic_aggregator`)

Multi-node system diagnostics with hardware monitoring, aggregation, and health checking via services.

**Features demonstrated:**
- Custom IDL messages with nested types (`DiagnosticEntry` + `KeyValue`)
- Service server and client pattern
- `spin_until_future_complete` for synchronous service calls
- `MultiThreadedExecutor` for parallel processing
- Multiple timers at different frequencies
- `TransientLocal` durability for late-joining subscribers
- Namespace isolation for subsystems (`/hw_subsystem/`, `/sensors/`, `/diagnostics/`)
- `std::mutex` for thread-safe state access
- YAML parameter loading

**Run:**
```bash
./diagnostic_aggregator --params-file config/diagnostic_aggregator.yaml
```

---

### 3. Adaptive Image Pipeline (`adaptive_image_pipeline`)

Adaptive-rate image publisher with a WaitSet-based synchronous receiver.

**Features demonstrated:**
- `WaitSet` synchronous polling model (non-callback)
- `take()` with `MessageInfo` for message metadata
- `KeepAll` QoS history policy
- Dynamic rate adaptation based on `get_subscription_count()`
- `WallRate` for rate-controlled loops
- `spin_some()` integration in manual loops
- `Clock::now()` for timestamping
- Multi-tier publishing (high-quality + thumbnail)
- Shared `DomainParticipant` between publisher and receiver
- Background thread for concurrent WaitSet polling

**Run:**
```bash
./adaptive_image_pipeline
```

---

### 4. State Machine Controller (`state_machine_controller`)

A robot controller with timer-driven states and service-based command interface.

**Features demonstrated:**
- Timer control: `cancel()`, `reset()`, `is_canceled()`, `is_ready()`
- Service-based state transitions (external command protocol)
- State machine pattern: IDLE → SCANNING → TRACKING → IDLE
- Multiple publishers for different state outputs
- Dynamic timer activation/deactivation per state
- `MultiThreadedExecutor` for parallel controller + command sender
- `Time` arithmetic for duration tracking
- Automatic state transitions with configurable thresholds

**Run:**
```bash
./state_machine_controller
```

---

## Custom IDL Messages

### `advanced_msgs/msg/VideoStreamStatus`
| Field | Type | Description |
|---|---|---|
| `header` | `std_msgs/Header` | Timestamp + frame_id |
| `stream_name` | `string` | Stream identifier |
| `width` / `height` | `int32` | Frame dimensions |
| `encoding` | `string` | Codec (e.g. "h265") |
| `frame_count` | `uint64` | Total frames processed |
| `bytes_total` | `uint64` | Total bytes transmitted |
| `fps_actual` / `fps_target` | `double` | FPS metrics |
| `latency_ms` | `double` | End-to-end latency |
| `bitrate_kbps` | `double` | Current bitrate |
| `is_keyframe` | `bool` | Keyframe indicator |
| `quality_level` | `uint8` | Quality metric (0–100) |
| `thumbnail` | `uint8[]` | Preview image |

### `advanced_msgs/msg/DiagnosticEntry`
| Field | Type | Description |
|---|---|---|
| `header` | `std_msgs/Header` | Timestamp + frame_id |
| `node_name` | `string` | Reporting node name |
| `hardware_id` | `string` | Hardware identifier |
| `level` | `uint8` | 0=OK, 1=WARN, 2=ERROR, 3=STALE |
| `message` | `string` | Status summary |
| `values` | `KeyValue[]` | Telemetry key-value pairs |

## Build

These examples are built automatically as part of the apps build:

```bash
# Build all apps (includes advanced)
./build_all.sh fastdds install
# or
./build_all.sh cyclonedds install
```
