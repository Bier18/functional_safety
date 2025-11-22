import rclpy
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.msg import DisplayTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from rclpy.action import ActionClient

class TrajectorySender(Node):
    #costruttore
    def __init__(self):
        # costruttore del nodo
        super().__init__("trajectory_sender")
        self.trajectory_sub = self.create_subscription(DisplayTrajectory,'/display_planned_path',self.send_trajectory,10)
        self.traj_client = ActionClient(self, FollowJointTrajectory, '/fr3_arm_controller/follow_joint_trajectory')

    def send_trajectory(self, msg: DisplayTrajectory):
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = msg.trajectory[-1].joint_trajectory
        self.traj_client.wait_for_server()
        future = self.traj_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("goal rejected")
            return
        self.get_logger().info("goal accepted")

def main():
    rclpy.init()
    trajectory_sender = TrajectorySender()
    rclpy.spin(trajectory_sender)
    rclpy.shutdown()

if __name__ == '__main__':
    main
