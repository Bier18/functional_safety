import rclpy
from rclpy.node import Node
import os
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import String
import csv
import hashlib

class Logger(Node):
    def __init__(self):
        super().__init__("logger")
        # sottoscrive il topic di log
        self.create_subscription(String, "/log_topic", self.save_info, 100)
        # memorizza il percorso di log
        #pkg_dir = get_package_share_directory('safe_controller')
        #self.target_file = os.path.join(pkg_dir, 'log_files', 'log.csv')
        self.target_file = '/ros2_ws/src/functional_safety/safe_controller/log_files/log.csv'
    
    def save_info(self, msg: String):

        # crea i campi evento tempo e firma
        evento = msg.data
        tempo = self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec * 1e-9
        data_str = f"{evento},{tempo}"
        firma = hashlib.sha256(data_str.encode('utf-8')).hexdigest()
        
        with open(self.target_file, mode='a', newline='') as f:
            writer = csv.writer(f)

            if not os.path.exists(self.target_file):
                writer.writerow(["evento", "tempo", "firma"])
            
            writer.writerow([evento, tempo, firma])



def main(args=None):
    rclpy.init(args=args)
    logger = Logger()
    rclpy.spin(logger)
    logger.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()