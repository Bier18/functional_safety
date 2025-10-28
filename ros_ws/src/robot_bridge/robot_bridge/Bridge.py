import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.msg import DisplayTrajectory, CollisionObject
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException
from safety_msgs.msg import Positions, SSM
from geometry_msgs.msg import Pose

class Bridge(Node) :
    def __init__(self):
        super().__init__('bridge_node')
        # server
        self._srs_stop_srv = self.create_service(SetBool, '/stop_robot', self.manage_movement)
        self._override_control_srv = self.create_service(SetBool,'/override_control',self.change_control)
        #subscribers
        self.joint_states_sub = self.create_subscription(JointState, '/franka/joint_states', self.save_state,10)
        self.trajectory_sub = self.create_subscription(DisplayTrajectory,'/display_planned_path',self.save_goal,10)
        self._obstacle_pos_sub = self.create_subscription(CollisionObject, '/collision_object',self.read_op_pose,10)
        #publishers
        self._human_monitoring_pub = self.create_publisher(Positions,'/human_monitoring',10)
        self._tcp_pose_pub = self.create_publisher(Positions,'/tcp_op_pose',10)
        self._ssm_pub = self.create_publisher(SSM,'/ssm',10)
        #action client
        self.traj_client = ActionClient(self, FollowJointTrajectory, '/fr3_arm_controller/follow_joint_trajectory')
        #inizializzazione variabili relative al franka
        self.joint_positions = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        self.joint_names = ['fr3_joint1', 'fr3_joint2', 'fr3_joint3', 'fr3_joint4', 'fr3_joint5', 'fr3_joint6', 'fr3_joint7']
        self.actual_time = Duration()
        self.trajectory = None
        self.tcp_pose_covariance = [0.0] * 36
        self.tcp_velcity_covariance = [0.0] * 36
        # DATI DA LEGGERE DA FILE YAML
        self.use_robot_velocity = False
        self.stop_time = 0.5
        self.reaction_time = 0.5
        self.safety_margin = 0.5
        self.tcp_to_op_max_speed = 0.5
        self.robot_stop_speed = 0.5
        #inizializzazione buffer e transform listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        #timer di ascolto della posizione
        self.timer = self.create_timer(1.0, self.send_tcp_and_op_pose)
        #variabili inerenti la posizione dell'operatore
        self.op_pose = Pose()
        self.op_pose_covariance = [0.0]*36
        self.op_velocity_covariance = [0.0]*36
        self.use_operator_velocity = False



    # stops the robot or makes it move
    def manage_movement(self,request,response):
        if(request.data):
            if self.joint_positions is None or self.joint_names is None:
                self.get_logger().warn("Joint states not received yet")
                return
            
            goal_msg = FollowJointTrajectory.Goal()
            goal_msg.trajectory.joint_names = self.joint_names
            point = JointTrajectoryPoint()
            point.positions = self.joint_positions
            point.velocities = [0.0] * len(self.joint_positions)
            point.accelerations = [0.0] * len(self.joint_positions)
            point.time_from_start = Duration(sec=1, nanosec=0)
            goal_msg.trajectory.points.append(point)
            self.traj_client.wait_for_server()
            future = self.traj_client.send_goal_async(goal_msg)
            future.add_done_callback(self.goal_response_callback)
            
            response.success = True
            response.message = "Robot Stopped"
        else:
            if not self.trajectory:
                self.get_logger().warn("No trajectory received yet!")
                response.success = False
                response.message = "No trajectory"
                return response
            
            goal_msg = FollowJointTrajectory.Goal()
            goal_msg.trajectory.joint_names = self.joint_names
            point = JointTrajectoryPoint()
            point.positions = self.trajectory[-1].joint_trajectory.points[-1].positions
            point.velocities = self.trajectory[-1].joint_trajectory.points[-1].velocities
            point.accelerations = self.trajectory[-1].joint_trajectory.points[-1].accelerations
            sec = self.trajectory[-1].joint_trajectory.points[-1].time_from_start.sec - self.actual_time.sec
            nanosec = self.trajectory[-1].joint_trajectory.points[-1].time_from_start.nanosec - self.actual_time.nanosec
            point.time_from_start.sec = sec if sec > 0 else 0
            point.time_from_start.nanosec = nanosec if nanosec > 0 else 0
            goal_msg.trajectory.points.append(point)

            self.traj_client.wait_for_server()
            future = self.traj_client.send_goal_async(goal_msg)
            future.add_done_callback(self.goal_response_callback)

            response.success = True
            response.message = "Robot Moving"
        
        return response
    
    #saves the current position
    def save_state(self, msg: JointState):
        self.joint_positions = msg.position
        if not self.trajectory:
            return
        min_dist = float('inf')
        for trajectory in self.trajectory:
            for pt in trajectory.joint_trajectory.points:
                dist = sum((p - q)**2 for p, q in zip(pt.positions, self.joint_positions))**0.5
                if dist < min_dist:
                    min_dist = dist
                    self.actual_time = pt.time_from_start
    
    #save the origianl goal
    def save_goal(self, msg: DisplayTrajectory):
        self.trajectory = msg.trajectory
    
    # callback for managing action server response
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("goal rejected")
            return
        self.get_logger().info("goal accepted")
    
    #read tcp and op pose and send it to state machine
    def send_tcp_and_op_pose(self):
        #legge la posizione del tcp
        try:
            t = self.tf_buffer.lookup_transform(
                'base',
                'fr3_hand_tcp',
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform base to fr3_hand_tcp: {ex}')
            return
        #CREA IL MESSAGGIO DA INVIARE AL TOPIC DELLA MODALITA' SRS E HG
        #imposta la posizione del tcp
        position_msg = Positions()
        position_msg.robot_pose.pose.position.x = t.transform.translation.x
        position_msg.robot_pose.pose.position.y = t.transform.translation.y
        position_msg.robot_pose.pose.position.z = t.transform.translation.z
        position_msg.robot_pose.pose.orientation = t.transform.rotation
        position_msg.robot_pose.covariance = self.tcp_pose_covariance
        #imposta la posizione dell'operatore
        position_msg.op_pose.pose = self.op_pose
        position_msg.op_pose.covariance = self.op_pose_covariance
        #pubblica il messaggio
        self._human_monitoring_pub.publish(position_msg)
        self._tcp_pose_pub.publish(position_msg)

        #CREA IL MESSAGGIO SSM
        ssm_msg = SSM()
        ssm_msg.use_robot_velocity = self.use_robot_velocity
        ssm_msg.use_operator_velocity = self.use_operator_velocity
        ssm_msg.operator_state.operator_pose.pose = self.op_pose
        ssm_msg.operator_state.operator_pose.covariance = self.op_pose_covariance
        ssm_msg.robot_state.robot_pose.pose.position.x = t.transform.translation.x
        ssm_msg.robot_state.robot_pose.pose.position.y = t.transform.translation.y
        ssm_msg.robot_state.robot_pose.pose.position.z = t.transform.translation.z
        ssm_msg.robot_state.robot_pose.pose.orientation = t.transform.rotation
        ssm_msg.robot_state.robot_pose.covariance = self.tcp_pose_covariance
        ssm_msg.stop_time = self.stop_time
        ssm_msg.reaction_time = self.reaction_time
        ssm_msg.safety_margin = self.safety_margin
        ssm_msg.tcp_to_op_max_speed = self.tcp_to_op_max_speed
        ssm_msg.robot_stop_speed = self.robot_stop_speed

        self._ssm_pub.publish(ssm_msg)
    
    #read operator pose
    def read_op_pose(self, msg: CollisionObject):
        self.op_pose = msg.pose
    
    #switch to/from manual control
    def change_control(self, request, response):
        if(request.data):
            print("Switch to manual mode")
            
            response.success = True
            response.message = "Manual guide ready"
        else:
            print("Switch to automatic control")

            response.success = True
            response.message = "Automatic guide ready"
        
        return response




def main():
    rclpy.init()
    bridge = Bridge()
    rclpy.spin(bridge)
    rclpy.shutdown()

if __name__ == '__main__':
    main()