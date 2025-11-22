import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from safety_msgs.msg import SSM
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from collections import deque
import numpy as np
from std_srvs.srv import SetBool
from std_msgs.msg import String

class SafeController(Node):
    def __init__(self):
        super().__init__("safe_controller")
        # comunicazione col bridge
        self.sensor_one = self.create_subscription(Pose, '/sensor_1',lambda msg: self.save_pose(msg, self.history_1),10)
        self.sensor_two = self.create_subscription(Pose, '/sensor_2', lambda msg: self.save_pose(msg, self.history_2),10)
        self.log_pub = self.create_publisher(String,'/log_topic',100)
        #richiesta di stop
        self.stop_robot = self.create_client(SetBool, '/stop_robot')
        # comunicazione con la state_machine
        self._ssm_pub = self.create_publisher(SSM,'/ssm',10)
        # DATI DA LEGGERE DA FILE YAML
        self.use_robot_velocity = False
        self.stop_time = 0.01
        self.reaction_time = 0.01
        self.safety_margin = 0.5
        self.tcp_to_op_max_speed = 1.6
        self.robot_stop_speed = 1.0
        self.tcp_pose_covariance = [0.0] * 36
        #inizializzazione buffer e transform listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        #timer di ascolto della posizione
        #self.timer = self.create_timer(1.0, self.elaborate_and_send)
        #variabili inerenti la posizione dell'operatore
        self.op_pose = Pose()
        self.op_pose_covariance = [0.0]*36
        self.op_velocity_covariance = [0.0]*36
        self.use_operator_velocity = False
        # storia delle rilevazioni
        self.history_1 = deque(maxlen=5)
        self.history_2 = deque(maxlen=5)
        # timer di invio dei dati
        self.send_data = self.create_timer(0.1,self.elaborate_and_send)
        # flag per ultime mediane ricevute
        self.last_dist_1 = 0.0
        self.last_dist_2 = 0.0
        # flag di rilevazione del fault
        self.fault_1 = False
        self.fault_2 = False
        # limiti di distanza massima rilevabili dai sensori
        self.max_len = 100
        self.min_dist = 1e-8
        self.max_change = 0.5
        self.max_sensor_diff = 0.3
        # flag di stop
        self.stop = False
        # fleg di log
        self.logged_1 = False
        self.logged_2 = False
    
    # salva le pose ridondanti
    def save_pose(self, msg: Pose, history):
        row = np.array([msg.position.x, msg.position.y, msg.position.z])
        history.append(row)
    
    def elaborate_and_send(self):
        # calcola le mediane
        if (len(self.history_1) < 5 or len(self.history_2) < 5):
            return
        else:
            #calcolo mediana 1
            mat_1 = np.vstack(self.history_1).T
            med_1 = np.median(mat_1, axis=1)
            #calcolo mediana 2
            mat_2 = np.vstack(self.history_2).T
            med_2 = np.median(mat_2, axis=1)
            # calcola le distanze
            dist_1 = np.linalg.norm(med_1)
            dist_2 = np.linalg.norm(med_2)
            # controllo sulle distanze
            if not self.min_dist <= dist_1 <= self.max_len: self.fault_1 = True
            if not self.min_dist <= dist_2 <= self.max_len: self.fault_2 = True
            # controllo di coerenza temporale
            if self.last_dist_1 != 0.0 and abs(dist_1 - self.last_dist_1) > self.max_change:
                self.fault_1 = True
            if self.last_dist_2 != 0.0 and abs(dist_2 - self.last_dist_2) > self.max_change:
                self.fault_2 = True
            # segnalazione del fault
            if self.fault_1 and not self.logged_1:
                self.send_log("[SSM] sensore 1 in fault")
                self.logged_1 = True 
                #manda med_2
            if self.fault_2 and not self.logged_2:
                self.send_log("[SSM] sensore 2 in fault")
                self.logged_2 = True
                #manda med_1
            
            self.last_dist_1 = dist_1
            self.last_dist_2 = dist_2
            
            if not self.fault_1 and not self.fault_2:
                # entrambi ok → invia il più conservativo
                if dist_1 < dist_2:
                    self.op_pose.position.x = med_1[0]
                    self.op_pose.position.y = med_1[1]
                    self.op_pose.position.z = med_1[2]
                else:
                    self.op_pose.position.x = med_2[0]
                    self.op_pose.position.y = med_2[1]
                    self.op_pose.position.z = med_2[2]
            elif not self.fault_1:
                # solo sensore 1 ok
                self.op_pose.position.x = med_1[0]
                self.op_pose.position.y = med_1[1]
                self.op_pose.position.z = med_1[2]
            elif not self.fault_2:
                # solo sensore 2 ok
                self.op_pose.position.x = med_2[0]
                self.op_pose.position.y = med_2[1]
                self.op_pose.position.z = med_2[2]
            else:
                # entrambi in fault → azione di sicurezza
                if not self.stop:
                    self.send_log("Entrambi i sensori in fault! Stop robot.")
                    stop_request = SetBool.Request()
                    stop_request.data = True
                    self.stop_robot.call_async(stop_request)
                    self.stop = True
                return
            
            #legge la posizione del tcp
            try:
                t = self.tf_buffer.lookup_transform(
                    'base',
                    'fr3_hand_tcp',
                    rclpy.time.Time())
            except TransformException as ex:
                self.send_log(
                    f'Could not transform base to fr3_hand_tcp: {ex}')
                return
            
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
    
    def send_log(self, string):
        log = String()
        log.data = string
        self.log_pub.publish(log)




def main(args=None):
    rclpy.init(args=args)
    safe_controller = SafeController()
    rclpy.spin(safe_controller)
    safe_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()