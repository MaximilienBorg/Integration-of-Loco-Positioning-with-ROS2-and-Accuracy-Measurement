import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import math
import time

class MockLPSNode(Node):
    def __init__(self):
        super().__init__('mock_lps_node')
        # On crée le "Topic" sur lequel le vrai LPS publiera plus tard
        self.publisher_ = self.create_publisher(PoseStamped, '/crazyflie/lps_pose', 10)
        self.timer = self.create_timer(0.1, self.timer_callback) # 10 Hz
        self.start_time = time.time()
        self.get_logger().info('Faux LPS démarré : Simulation de vol circulaire activée !')

    def timer_callback(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map' # C'est le repère global de la pièce

        # Simulation mathématique d'un vol en cercle (rayon de 1m)
        t = time.time() - self.start_time
        msg.pose.position.x = 1.0 * math.cos(t)
        msg.pose.position.y = 1.0 * math.sin(t)
        msg.pose.position.z = 1.0 # Le drone vole à 1 mètre d'altitude

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MockLPSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
