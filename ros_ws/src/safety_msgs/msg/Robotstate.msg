# RobotState.msg

# Timestamp indicating when the data was generated
builtin_interfaces/Time stamp

# Pose of the robot's TCP (tool center point) in a specific reference frame
# Includes header.frame_id and timestamp for proper coordination
geometry_msgs/PoseStamped tcp_pose

# Velocity of the robot's TCP (linear + angular) in a specific reference frame
# Includes header.frame_id and timestamp
geometry_msgs/TwistStamped tcp_velocity

# Boolean flag indicating whether the data is valid and can be safely used
# True if tcp_pose and tcp_velocity are reliable; False otherwise
bool valid