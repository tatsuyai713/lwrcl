# SampleData.msg
# A simple ROS 2-style message demonstrating that a .msg file dropped here
# is automatically converted into IDL by apps/scripts/msg_to_idl.py and then
# fed to the active DDS backend's IDL generator (fastddsgen / idlc).

# Status constants (uint8 named values become IDL `const`).
uint8 STATUS_OK    = 0
uint8 STATUS_WARN  = 1
uint8 STATUS_ERROR = 2

# Standard ROS 2 header (auto resolves to std_msgs::msg::Header).
std_msgs/Header header

# Monotonically-increasing sequence number assigned by the publisher.
int32 sequence_number

# Sensor reading.
float64 value

# Human-readable label.
string label

# Status code; one of STATUS_* above.
uint8 status
