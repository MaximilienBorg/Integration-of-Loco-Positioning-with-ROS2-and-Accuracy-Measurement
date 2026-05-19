import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import csv
import os
import time
from datetime import datetime

class LoggerNode(Node):
    def __init__(self):
        super().__init__('logger_node')
        
        # Variables de stockage temporaire
        self.drone_pose = None
        self.turtle_pose = None
        self.turtle_vel_linear = 0.0
        self.turtle_vel_angular = 0.0
        
        # Création des Subscribers
        self.sub_drone = self.create_subscription(PoseStamped, '/crazyflie/lps_pose', self.drone_cb, 10)
        self.sub_turtle = self.create_subscription(Odometry, '/tb4/odom', self.turtle_cb, 10)
        
        # Préparation du fichier CSV
        log_dir = os.path.expanduser('~/ros2_ws/flight_logs')
        os.makedirs(log_dir, exist_ok=True)
        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_filename = os.path.join(log_dir, f'follower_log_{timestamp_str}.csv')
        
        self.csv_file = open(self.csv_filename, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        
        # --- NOUVELLES COLONNES AJOUTÉES ICI ---
        self.csv_writer.writerow(['Temps_s', 'Drone_X', 'Drone_Y', 'Drone_Z', 'Turtle_X', 'Turtle_Y', 'Turtle_Z', 'Distance_m', 'Turtle_V_Lin', 'Turtle_V_Ang'])
        
        self.start_time = time.time()
        self.is_logging = True
        
        # Timer pour enregistrer à 10 Hz (10 fois par seconde)
        self.timer = self.create_timer(0.1, self.log_timer_cb)
        self.get_logger().info(f"Logger démarré. Fichier : {self.csv_filename}")

    def drone_cb(self, msg):
        self.drone_pose = msg.pose.position

    def turtle_cb(self, msg):
        self.turtle_pose = msg.pose.pose.position
        # --- EXTRACTION DE LA VITESSE DU TURTLEBOT ---
        self.turtle_vel_linear = msg.twist.twist.linear.x
        self.turtle_vel_angular = msg.twist.twist.angular.z

    def log_timer_cb(self):
        # On écrit une ligne seulement si on a reçu des données des deux robots
        if self.is_logging and self.drone_pose and self.turtle_pose:
            t = time.time() - self.start_time
            
            # Calcul de la distance 3D avec l'offset de (1.0, 1.0)
            dx = self.drone_pose.x - (self.turtle_pose.x + 1.0)
            dy = self.drone_pose.y - (self.turtle_pose.y + 1.0)
            dz = self.drone_pose.z - self.turtle_pose.z
            dist = (dx**2 + dy**2 + dz**2)**0.5
            
            # Écriture dans le fichier CSV
            self.csv_writer.writerow([
                round(t, 2),
                round(self.drone_pose.x, 3), round(self.drone_pose.y, 3), round(self.drone_pose.z, 3),
                round(self.turtle_pose.x, 3), round(self.turtle_pose.y, 3), round(self.turtle_pose.z, 3),
                round(dist, 3),
                round(self.turtle_vel_linear, 3),  # Nouvelle donnée : Vitesse m/s
                round(self.turtle_vel_angular, 3)  # Nouvelle donnée : Rotation rad/s
            ])

def main(args=None):
    rclpy.init(args=args)
    node = LoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Arrêt de l'enregistrement demandé (Ctrl+C).")
    finally:
        node.csv_file.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
