import rclpy
from rclpy.node import Node
import yaml
import os
from robot import Robot
from joint import Joint
from rclpy.qos import QoSProfile

class Bridge(Node):
    def __init__(self, robot_name: str, yaml_file: str):
        super().__init__('bridge_node')

        # --- Load YAML ---
        with open(yaml_file, 'r') as f:
            self.config = yaml.safe_load(f)

        # --- Initialize Robot and Joints ---
        joint_names = [j['name'] for j in self.config['/**']['bridge_params']]
        self.robot = Robot(robot_name, joint_names)

        # --- Setup subscribers and publishers ---
        self.subscriptions = {}
        self.publishers = {}

        for joint_conf in self.config['/**']['bridge_params']:
            name = joint_conf['name']

            # ROS topic for reading state
            ros_topic_conf = joint_conf.get('ros_topic')
            if ros_topic_conf:
                state_topic = ros_topic_conf['robot_topic']
                msg_type = self._import_msg_type(ros_topic_conf['robot_msg_type'])
                qos = QoSProfile(depth=10)
                self.subscriptions[name] = self.create_subscription(
                    msg_type,
                    state_topic,
                    lambda msg, j_name=name: self._read_joint_state(j_name, msg, ros_topic_conf['mapping']),
                    qos
                )

            # ROS topic for sending control (to controller)
            control_topic_conf = joint_conf.get('control_topic')
            if control_topic_conf:
                ctrl_topic = control_topic_conf['controller_topic']
                ctrl_msg_type = self._import_msg_type(control_topic_conf['controller_msg_type'])
                self.publishers[name] = self.create_publisher(ctrl_msg_type, ctrl_topic, 10)

    def _import_msg_type(self, type_str: str):
        """
        Import ROS message class dynamically from string e.g. 'sensor_msgs/JointState'
        """
        package, msg = type_str.split('/')
        module = __import__(f"{package}.msg", fromlist=[msg])
        return getattr(module, msg)

    def _read_joint_state(self, joint_name: str, msg, mapping: dict):
        """
        Aggiorna il digital twin con i dati del giunto.
        """
        pos = getattr(msg, mapping['position'])
        vel = getattr(msg, mapping['velocity'])
        eff = getattr(msg, mapping['effort'])
        self.robot.update_joint(joint_name, pos, vel, eff)

    def send_control(self, joint_name: str, position: float = None, velocity: float = None, effort: float = None):
        """
        Pubblica i comandi sul topic controller per il giunto specifico.
        """
        joint_conf = next(j for j in self.config['/**']['bridge_params'] if j['name'] == joint_name)
        control_topic_conf = joint_conf['control_topic']
        msg_type = self._import_msg_type(control_topic_conf['controller_msg_type'])
        msg = msg_type()
        mapping = control_topic_conf['mapping']
        if position is not None:
            setattr(msg, mapping['position'], position)
        if velocity is not None:
            setattr(msg, mapping['velocity'], velocity)
        if effort is not None:
            setattr(msg, mapping['effort'], effort)
        self.publishers[joint_name].publish(msg)

def main(args=None):
    rclpy.init(args=args)
    bridge = Bridge()
    rclpy.spin(bridge)
    bridge.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()