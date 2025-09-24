import rclpy
from rclpy.node import Node
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import JointState
from .Robot import Robot
from .Joint import Joint


class Bridge(Node):

    def __init__(self):
        super().__init__("bridge_node")

        # read the configuration file
        with open(os.path.join(get_package_share_directory('robot_bridge'), 'config', 'bridge.yaml')) as f:
            self.config = yaml.safe_load(f)

        # initialize dictionaries
        self.ros_subscription = {}
        self.ros_publisher = {}
        self.controller_subscription = {}
        self.controller_publisher = {}

        # initialize digital twin
        joint_list = []

        for joint in self.config['/**']['robot_joints']:
            # --- ROS subscription (robot state)
            ros_topic_conf = joint.get('ros_topic')
            if ros_topic_conf:
                message = self.import_msg(ros_topic_conf['robot_msg_type'])
                topic = ros_topic_conf['robot_topic']
                self.ros_subscription[joint['name']] = self.create_subscription(
                    message,
                    topic,
                    lambda msg, name=joint['name']: self.save_and_translate(msg, name),
                    10
                )
                # publisher verso standard JointState
                self.ros_publisher[joint['name']] = self.create_publisher(
                    JointState,
                    f"/{joint['name']}/state",
                    10
                )

            # --- Control subscription
            ctrl_topic_conf = joint.get('control_topic')
            if ctrl_topic_conf:
                ctrl_msg_type = self.import_msg(ctrl_topic_conf['controller_msg_type'])
                ctrl_topic = ctrl_topic_conf['controller_topic']
                self.controller_subscription[joint['name']] = self.create_subscription(
                    ctrl_msg_type,
                    ctrl_topic,
                    lambda msg, name=joint['name']: self.imperio(msg, name),
                    10
                )
                # publisher verso robot
                robot_msg_type = self.import_msg(ctrl_topic_conf['robot_msg_type'])
                robot_topic = ctrl_topic_conf['robot_topic']
                self.controller_publisher[joint['name']] = self.create_publisher(
                    robot_msg_type,
                    robot_topic,
                    10
                )

            joint_list.append(joint['name'])

        self.robot = Robot(self.config['/**']['robot_name'], joint_list)

    def import_msg(self, msg_type: str):
        parts = msg_type.split('/')
        if len(parts) < 2:
            raise ValueError(f"Tipo messaggio invalido: '{msg_type}'")
        package = parts[0]
        msg = parts[-1]  # prende l’ultimo elemento come nome del messaggio
        module = __import__(f"{package}.msg", fromlist=[msg])
        return getattr(module, msg)

    def imperio(self, msg, name: str):
        joint_conf = next((j for j in self.config['/**']['robot_joints'] if j['name'] == name), None)
        if not joint_conf:
            return
        ctrl_conf = joint_conf['control_topic']
        mapping = ctrl_conf['mapping']

        msg_type = self.import_msg(ctrl_conf['robot_msg_type'])
        robot_msg = msg_type()

        self.set_field(robot_msg, mapping['position_setpoint'], msg.position_setpoint)
        self.set_field(robot_msg, mapping['velocity_setpoint'], msg.velocity_setpoint)
        self.set_field(robot_msg, mapping['effort_setpoint'], msg.effort_setpoint)

        self.controller_publisher[name].publish(robot_msg)

    def save_and_translate(self, msg, name: str):
        joint_conf = next(j for j in self.config['/**']['robot_joints'] if j['name'] == name)
        mapping = joint_conf['ros_topic']['mapping']

        pos = self.get_nested_field(msg, mapping['position'], None)
        vel = self.get_nested_field(msg, mapping['velocity'], None)
        eff = self.get_nested_field(msg, mapping['effort'], None)

        # aggiorna il digital twin
        self.robot.update_joint(name,pos,vel,eff)

        # costruisci un JointState standardizzato
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = [name]
        js.position = [pos] if pos is not None else []
        js.velocity = [vel] if vel is not None else []
        js.effort = [eff] if eff is not None else []

        self.ros_publisher[name].publish(js)
    
    def set_field(self,obj, field_path, value):
        parts = field_path.split('.')
        for attr in parts[:-1]:
            obj = getattr(obj, attr)
        setattr(obj, parts[-1], value)
    
    def get_nested_field(self,obj, field_path, default=None):
        fields = field_path.split('.')
        for field in fields:
            obj = getattr(obj, field, default)
            if obj is default:
                break
        return obj


def main(args=None):
    rclpy.init(args=args)
    bridge = Bridge()
    rclpy.spin(bridge)
    bridge.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()