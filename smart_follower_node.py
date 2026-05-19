import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import cflib.crtp
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
from cflib.positioning.position_hl_commander import PositionHlCommander
import threading
import time

URI = 'radio://0/30/2M/E7E7E7E7E7'

class SmartFollowerNode(Node):
    def __init__(self):
        super().__init__('smart_follower_node')
        
        self.publisher_ = self.create_publisher(PoseStamped, '/crazyflie/lps_pose', 10)
        self.sub_turtle = self.create_subscription(Odometry, '/tb4/odom', self.turtle_cb, 10)
        
        # --- LE SECRET EST ICI ---
        # Position physique où tu dois TOUJOURS poser le TurtleBot avant de l'allumer
        self.TB_START_X = 1.0
        self.TB_START_Y = 1.0
        
        # Cible initiale (se mettra à jour avec l'odométrie)
        self.target_x = self.TB_START_X
        self.target_y = self.TB_START_Y
        
        cflib.crtp.init_drivers()
        self.get_logger().info(f"Connexion à {URI}...")
        self.scf = SyncCrazyflie(URI)
        self.scf.open_link()
        self.get_logger().info("✅ Connecté !")

        self.log_conf = LogConfig(name='Position', period_in_ms=100)
        self.log_conf.add_variable('stateEstimate.x', 'float')
        self.log_conf.add_variable('stateEstimate.y', 'float')
        self.log_conf.add_variable('stateEstimate.z', 'float')
        self.scf.cf.log.add_config(self.log_conf)
        self.log_conf.data_received_cb.add_callback(self._publish_ros2_data)
        self.log_conf.start()

        self.flight_thread = threading.Thread(target=self.flight_sequence)
        self.flight_thread.start()

    def turtle_cb(self, msg):
        # On additionne l'odométrie (qui commence à 0) avec la position de départ (2,2)
        self.target_x = msg.pose.pose.position.x + self.TB_START_X
        self.target_y = msg.pose.pose.position.y + self.TB_START_Y

    def flight_sequence(self):
        self.get_logger().info("⏳ Stabilisation (3 secondes)...")
        time.sleep(3)
        
        # 1. DECOLLAGE VERTICAL DEPUIS N'IMPORTE OÙ
        self.get_logger().info("🚀 DÉCOLLAGE vertical (1 mètre)...")
        with PositionHlCommander(self.scf, default_height=1.0) as pc:
            time.sleep(2)
            
            # 2. RENDEZ-VOUS AVEC LE TURTLEBOT
            self.get_logger().info("🎯 Vol en douceur vers le TurtleBot...")
            # pc.go_to calcule une trajectoire propre pour rejoindre le robot
            pc.go_to(self.target_x, self.target_y, 1.0)
            time.sleep(1)
            
            # 3. MODE SUIVI DYNAMIQUE
            self.get_logger().info("🔄 Mode Suivi Activé ! Tu peux conduire le TurtleBot.")
            while rclpy.ok():
                # Geofencing de sécurité (Le drone ne dépassera jamais ces coordonnées, même si le robot le fait)
                safe_x = max(0.0, min(3.0, self.target_x))
                safe_y = max(0.0, min(3.0, self.target_y))
                safe_z = 1.0 
                
                self.scf.cf.commander.send_position_setpoint(safe_x, safe_y, safe_z, 0)
                time.sleep(0.05) # Rafraîchissement 20 fois par seconde pour un suivi très fluide
                
    def _publish_ros2_data(self, timestamp, data, logconf):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = data['stateEstimate.x']
        msg.pose.position.y = data['stateEstimate.y']
        msg.pose.position.z = data['stateEstimate.z']
        msg.pose.orientation.w = 1.0
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SmartFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Arrêt d'urgence. Atterrissage...")
    
    node.scf.close_link()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
