# Vector3.msg
# A simple custom type defined as a sibling .msg in this same package.
# Other .msg files in this package can reference it by writing
# "ros_msg_mixed_example/Vector3" - the build system converts that into
# an IDL include of the matching auto-generated .idl.
float64 x
float64 y
float64 z
