# RobotSafetyParams.msg

# Worst-case stop time of the robot [seconds]
float32 stop_time       

# Additional safety margin (extra space to avoid collisions) [meters]
float32 safety_margin    

# Maximum speed of the robot towards the operator [m/s]
float32 max_speed        

# Uncertainty of the robot's position measurement (x, y, z) [meters]
geometry_msgs/Vector3 position_uncertainty_robot  

# Uncertainty of the robot's velocity measurement (vx, vy, vz) [m/s]
geometry_msgs/Vector3 velocity_uncertainty_robot  

# Estimated uncertainty of the operator's position (x, y, z) [meters]
geometry_msgs/Vector3 position_uncertainty_operator  

# Estimated uncertainty of the operator's velocity (vx, vy, vz) [m/s]
geometry_msgs/Vector3 velocity_uncertainty_operator  