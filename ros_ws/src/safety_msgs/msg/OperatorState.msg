# OperatorState.msg

# Timestamp indicating when the data was generated
builtin_interfaces/Time stamp

# Operator's pose (position + orientation) in a specific reference frame
# Includes a header with frame_id and timestamp for proper coordination
geometry_msgs/PoseStamped pose       

# Operator's velocity (linear + angular) in a specific reference frame
# Includes a header with frame_id and timestamp
geometry_msgs/TwistStamped velocity  

# Boolean flag indicating whether the data is valid and can be safely used
# True if pose and velocity are reliable; False otherwise
bool valid                            