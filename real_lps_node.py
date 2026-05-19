import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import cflib.crtp
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
from cflib.positioning.position_hl_commander import PositionHlCommander
import threading
import time

# /!\ REMPLACE CECI PAR LA VRAIE ADRESSE DE TON DRONE
URI = 'radio://0/30/2M/E7E7E7E7E7' 

class RealLpsNode(Node):
    def __init__(self):
        super().__init__('real_lps_node')
        self.publisher_ = self.create_publisher(PoseStamped, '/crazyflie/lps_pose', 10)
        
        cflib.crtp.init_drivers()
        self.get_logger().info(f"Connexion à {URI}...")
        
        # On utilise SyncCrazyflie pour pouvoir utiliser le Commander
        self.scf = SyncCrazyflie(URI)
        self.scf.open_link()
        self.get_logger().info("✅ Connecté ! Configuration du LPS...")

        # --- PARTIE 1 : LECTURE DE LA POSITION (TELEMETRIE) ---
        self.log_conf = LogConfig(name='Position', period_in_ms=100)
        self.log_conf.add_variable('stateEstimate.x', 'float')
        self.log_conf.add_variable('stateEstimate.y', 'float')
        self.log_conf.add_variable('stateEstimate.z', 'float')

        self.scf.cf.log.add_config(self.log_conf)
        self.log_conf.data_received_cb.add_callback(self._publish_ros2_data)
        self.log_conf.start()

        # --- PARTIE 2 : CHOREGRAPHIE DE VOL (Dans un thread séparé) ---
        self.flight_thread = threading.Thread(target=self.flight_sequence)
        self.flight_thread.start()

    def flight_sequence(self):
        self.get_logger().info("⏳ Stabilisation du LPS (3 secondes)...")
        time.sleep(3)
        
        self.get_logger().info("🚀 DECOLLAGE IMMINENT !")
        with PositionHlCommander(self.scf, default_height=1.0, default_velocity=0.15) as pc:
            self.get_logger().info("Étape 1 : Maintien en (1, 1, 1)")
            time.sleep(2)
            
            self.get_logger().info("➡️ Étape 2 : Vol vers (3, 1, 1)")
            pc.go_to(3.0, 1.0, 1.0)
            time.sleep(2)
            
            self.get_logger().info("🛬 Étape 3 : Atterrissage")
            
        self.get_logger().info("🏁 Mission terminée. Moteurs coupés.")

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
    node = RealLpsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    # Nettoyage à la fermeture
    node.scf.close_link()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
