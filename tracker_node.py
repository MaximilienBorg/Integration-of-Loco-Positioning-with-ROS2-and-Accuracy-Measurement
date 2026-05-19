import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import math

class TrackerNode(Node):
    def __init__(self):
        super().__init__('tracker_node')
        
        # Initialisation des variables de position
        self.drone_pos = None
        self.turtlebot_pos = None

        # Abonnements aux positions (Drone et Turtlebot)
        self.sub_drone = self.create_subscription(
            PoseStamped, '/crazyflie/lps_pose', self.drone_callback, 10)
            
        self.sub_turtlebot = self.create_subscription(
            Odometry, '/tb4/odom', self.turtlebot_callback, 10)

        # Création du Publisher pour la distance (C'EST ICI QUE CA CRASHAIT)
        self.distance_pub = self.create_publisher(Float64, '/tracking/distance', 10)

        # Timer pour calculer la distance toutes les secondes
        self.timer = self.create_timer(1.0, self.compute_distance)
        self.get_logger().info("Nœud de Tracking démarré : Publication sur /tracking/distance")

    def drone_callback(self, msg):
        self.drone_pos = msg.pose.position

    def turtlebot_callback(self, msg):
        self.turtlebot_pos = msg.pose.pose.position

    def compute_distance(self):
        if self.drone_pos is not None and self.turtlebot_pos is not None:
            # Calcul de la distance 3D
            dx = self.drone_pos.x - self.turtlebot_pos.x
            dy = self.drone_pos.y - self.turtlebot_pos.y
            dz = self.drone_pos.z - self.turtlebot_pos.z
            distance = math.sqrt(dx**2 + dy**2 + dz**2)
            
            # Publication sur le Topic ROS 2
            msg_dist = Float64()
            msg_dist.data = distance
            self.distance_pub.publish(msg_dist)
            
            # Affichage dans le terminal pour le confort visuel
            self.get_logger().info(f"Distance calculée et publiée : {distance:.2f} m")

def main(args=None):
    rclpy.init(args=args)
    node = TrackerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
