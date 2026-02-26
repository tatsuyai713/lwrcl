# Latency Benchmarks

Intra-process ping-pong latency measurement for 7 backend combinations:

- **lwrcl + CycloneDDS**
- **lwrcl + FastDDS**
- **lwrcl + Adaptive AUTOSAR (vsomeip)** (`ARA_COM_EVENT_BINDING=vsomeip`)
- **lwrcl + Adaptive AUTOSAR (DDS)** (`ARA_COM_EVENT_BINDING=dds`)
- **lwrcl + Adaptive AUTOSAR (iceoryx)** (`ARA_COM_EVENT_BINDING=iceoryx`)
- **ROS 2 Humble + CycloneDDS** (`rmw_cyclonedds_cpp`)
- **ROS 2 Humble + FastDDS** (`rmw_fastrtps_cpp`)

## Methodology

- Single node: ping publisher, pong publisher, ping subscriber, pong subscriber
- 1 ms wall timer fires each ping with current `steady_clock` timestamp in payload
- Pong subscriber echoes message back immediately
- Ping subscriber receives echo and computes one-way latency = (t2 − t1) / 2
- 200 warm-up samples discarded; 1000 samples collected
- Results reported as median of 5 independent runs

## Files

```
src/
  lwrcl_latency.cpp   — lwrcl version (uses msg->data() accessor)
  ros2_latency.cpp    — ROS 2 version (uses msg->data field)
build_lwrcl.sh        — Build lwrcl binaries (CycloneDDS + FastDDS)
build_ros2.sh         — Build ROS 2 binary (single binary, rmw selected at runtime)
run_all.sh            — Run all 4 patterns and print summary table
bin/                  — Built binaries (created by build scripts)
```

## Usage

### 1. Build

```bash
# Build lwrcl binaries (requires lwrcl installed for each backend)
./benchmarks/latency/build_lwrcl.sh

# Build ROS 2 binary (requires /opt/ros/humble)
./benchmarks/latency/build_ros2.sh
```

### 2. Run

```bash
# Run all 4 patterns (5 runs each, summary table at end)
./benchmarks/latency/run_all.sh

# Run with custom number of runs (e.g. 10)
./benchmarks/latency/run_all.sh 10
```

### 3. Run individual patterns

```bash
# lwrcl + CycloneDDS
LD_LIBRARY_PATH=/opt/iceoryx/lib:/opt/cyclonedds/lib:/opt/cyclonedds-libs/lib \
  ./benchmarks/latency/bin/lat_lwrcl_cyclone

# lwrcl + FastDDS
LD_LIBRARY_PATH=/opt/fast-dds/lib:/opt/fast-dds-libs/lib \
  ./benchmarks/latency/bin/lat_lwrcl_fastdds

# lwrcl + Adaptive AUTOSAR (SOME/IP transport)
ARA_COM_EVENT_BINDING=vsomeip \
LD_LIBRARY_PATH=/opt/autosar-ap-libs/lib:/opt/autosar_ap/lib:/opt/cyclonedds/lib:/opt/iceoryx/lib:/opt/vsomeip/lib \
  ./benchmarks/latency/bin/lat_lwrcl_autosar

# lwrcl + Adaptive AUTOSAR (DDS transport)
ARA_COM_EVENT_BINDING=dds \
LD_LIBRARY_PATH=/opt/autosar-ap-libs/lib:/opt/autosar_ap/lib:/opt/cyclonedds/lib:/opt/iceoryx/lib:/opt/vsomeip/lib \
  ./benchmarks/latency/bin/lat_lwrcl_autosar

# lwrcl + Adaptive AUTOSAR (iceoryx transport — requires RouDi)
ARA_COM_EVENT_BINDING=iceoryx \
LD_LIBRARY_PATH=/opt/autosar-ap-libs/lib:/opt/autosar_ap/lib:/opt/cyclonedds/lib:/opt/iceoryx/lib:/opt/vsomeip/lib \
  ./benchmarks/latency/bin/lat_lwrcl_autosar

# ROS 2 + CycloneDDS
source /opt/ros/humble/setup.bash
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  ./benchmarks/latency/bin/lat_ros2

# ROS 2 + FastDDS
source /opt/ros/humble/setup.bash
RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  ./benchmarks/latency/bin/lat_ros2
```

## Notes

- In Docker containers, **p99 is dominated by OS scheduler jitter** (~450–625 µs for all
  backends). p50 and p90 are more representative of typical middleware behavior.
- The ROS 2 binary is a single executable; the backend is selected at runtime via
  `RMW_IMPLEMENTATION` environment variable.
