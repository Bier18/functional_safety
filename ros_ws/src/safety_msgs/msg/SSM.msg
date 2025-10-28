# SafetyMonitorData.msg
bool use_robot_velocity
bool use_operator_velocity
safety_msgs/OperatorState operator_state
safety_msgs/RobotState robot_state
float64 stop_time
float64 reaction_time
float64 safety_margin #defined in standard ISO 13855
float64 tcp_to_op_max_speed
float64 robot_stop_speed