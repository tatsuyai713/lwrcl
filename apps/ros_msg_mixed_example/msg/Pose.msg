# Pose.msg
# Mixes:
#   - a sibling type defined in another .msg of this package (Vector3)
#   - an externally-defined IDL type from the ros-data-types package
#     (builtin_interfaces/Time)
#
# Same-package types are referenced by bare name (ROS 2 convention).
# Other packages use ``pkg/Type`` form. The build pipeline auto-resolves
# both into the corresponding .idl includes.

# External IDL type, referenced .msg-style (cross-package).
builtin_interfaces/Time stamp

# Sibling .msg type in this same package - bare name, ROS 2 style.
Vector3 position

# Inline primitives.
float64 yaw
