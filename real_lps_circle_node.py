import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import cflib.crtp
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
from cflib.positioning.position_hl_commander import PositionHlCommander
import threading
import time
import math

# L'adresse radio de ton drone
URI = 'radio://0/30/2M/E7E7E7E7E7' 

class RealLpsCircleNode(Node):
    def __init__(self):
        super().__init__('real_lps_circle_node')
        self.publisher_ = self.create_publisher(PoseStamped, '/crazyflie/lps_pose', 10)
        
        cflib.crtp.init_drivers()
        self.get_logger().info(f"Connexion à {URI}...")
        
        self.scf = SyncCrazyflie(URI)
        self.scf.open_link()
        self.get_logger().info("✅ Connecté ! Configuration du LPS...")

        # 1. TÉLÉMÉTRIE (Envoi des données à ROS 2)
        self.log_conf = LogConfig(name='Position', period_in_ms=100)
        self.log_conf.add_variable('stateEstimate.x', 'float')
        self.log_conf.add_variable('stateEstimate.y', 'float')
        self.log_conf.add_variable('stateEstimate.z', 'float')

        self.scf.cf.log.add_config(self.log_conf)
        self.log_conf.data_received_cb.add_callback(self._publish_ros2_data)
        self.log_conf.start()

        # 2. PILOTE AUTOMATIQUE
        self.flight_thread = threading.Thread(target=self.flight_sequence)
        self.flight_thread.start()

    def flight_sequence(self):
        self.get_logger().info("⏳ Stabilisation du LPS (3 secondes)...")
        time.sleep(3)
        
        self.get_logger().info("🚀 DECOLLAGE IMMINENT depuis la position actuelle !")
        with PositionHlCommander(self.scf, default_height=1.0) as pc:
            time.sleep(2)
            
            self.get_logger().info("🎯 Repositionnement au centre de la zone (2, 2, 1)...")
            pc.go_to(2.0, 2.0, 1.0)
            time.sleep(2)
            
            self.get_logger().info("➡️ Déplacement vers le bord du cercle (3, 2, 1)...")
            pc.go_to(3.0, 2.0, 1.0)
            time.sleep(2)

            self.get_logger().info("🔄 Début de la chorégraphie (3 cercles)...")
            center_x = 2.0
            center_y = 2.0
            radius = 1.0
            altitude = 1.0

            for degree in range(360 * 2):
                rad = math.radians(degree)
                x = center_x + radius * math.cos(rad)
                y = center_y + radius * math.sin(rad)
                
                self.scf.cf.commander.send_position_setpoint(x, y, altitude, 0)
                time.sleep(0.02)

            # ... fin de la boucle for (les cercles) ...

            self.get_logger().info("⬅️ Retour au centre (2, 2, 1)...")
            pc.go_to(2.0, 2.0, 1.0)
            time.sleep(2)
            
            # --- MODIFICATION 1 : Stabilisation avant atterrissage ---
            self.get_logger().info("⚖️ Stabilisation en l'air (2 secondes)...")
            time.sleep(2)
            
            self.get_logger().info("🛬 Début de l'atterrissage automatique...")
            
        # Le fait de sortir du bloc 'with' au-dessus donne l'ordre d'atterrir.
        # --- MODIFICATION 2 : Attente de la fin de la descente ---
        self.get_logger().info("⏳ En attente du contact avec le sol (4 secondes)...")
        time.sleep(4)
            
        self.get_logger().info("🏁 Mission terminée. Moteurs coupés.")
    # C'est cette fonction qui avait disparu !
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
    node = RealLpsCircleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.scf.close_link()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
