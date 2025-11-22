import rclpy
from rclpy.node import Node
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

class CollisionPublisher(Node):
    def __init__(self):
        super().__init__('collision_publisher')
        self.pub = self.create_publisher(CollisionObject, '/collision_object', 10)
        self.timer = self.create_timer(1, self.publish_object)
        self.step = 0.1
        self.start = 3.0

    def publish_object(self):
        co = CollisionObject()
        co.id = 'operator'
        co.header.frame_id = 'base'
        co.operation = CollisionObject.ADD

        cyl = SolidPrimitive()
        cyl.type = SolidPrimitive.CYLINDER
        cyl.dimensions = [1.75, 0.25]  # [height, radius]

        pose = Pose()
        pose.position.x = self.start
        pose.position.y = 0.0
        pose.position.z = 0.875  # metà altezza
        pose.orientation.w = 1.0

        co.primitives.append(cyl)
        co.primitive_poses.append(pose)

        self.pub.publish(co)
        self.start -= self.step

def main(args=None):
    rclpy.init(args=args)
    node = CollisionPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
