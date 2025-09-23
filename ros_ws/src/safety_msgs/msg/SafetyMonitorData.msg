# SafetyMonitorData.msg

# Timestamp indicating when the data package was generated
builtin_interfaces/Time stamp

# Operator state (pose + velocity + validity)
safety_msgs/OperatorState operator_state

# Robot state (TCP pose + velocity + validity)
safety_msgs/RobotState robot_state

# Robot safety parameters (stop time, uncertainties, etc.)
safety_msgs/SafetyParams robot_params

# Boolean flag indicating whether the whole message is valid
# True if all contained data are reliable; False otherwise
bool valid