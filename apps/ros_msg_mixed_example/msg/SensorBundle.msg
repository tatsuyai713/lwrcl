# SensorBundle.msg
# Top-level message used by the publisher/subscriber example.
#
# Demonstrates that within a single .msg file you can freely mix:
#   * primitive ROS 2 types (string, int32, float32[], ...)
#   * other .msg files in the same package (Pose -> Pose.msg here)
#   * external types whose canonical definition lives in IDL inside
#     ros-data-types (std_msgs/Header, geometry_msgs/Point) - the user
#     references them with the same ROS 2 "pkg/Type" syntax.
#
# No #include lines are needed in the .msg; msg_to_idl.py emits the
# correct .idl includes automatically for the generated .idl file.

# Status constants
uint8 STATUS_UNKNOWN = 0
uint8 STATUS_OK      = 1
uint8 STATUS_FAULT   = 2

# Externally-defined IDL type (from ros-data-types).
std_msgs/Header header

# Sibling .msg type - bare name (ROS 2 convention for same package).
Pose pose

# Bounded sequence of an external IDL type.
geometry_msgs/Point[<=8] waypoints

# Plain ROS 2 primitives and arrays.
float32[3] orientation_xyz
string description
uint8 status
