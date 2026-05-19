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

class RealLpsFollowerNode(Node):
    def __init__(self):
        super().__init__('real_lps_follower_node')
        
        # Publisher pour envoyer la position du drone
        self.publisher_ = self.create_publisher(PoseStamped, '/crazyflie/lps_pose', 10)
        
        # Subscriber pour ÉCOUTER le TurtleBot (attention, on utilise bien /tb4/odom !)
        self.sub_turtle = self.create_subscription(Odometry, '/tb4/odom', self.turtle_cb, 10)
        
        # Variables de ciblage (On initialise au centre d'offset 1,1)
        self.target_x = 1.0
        self.target_y = 1.0
        
        # Connexion au Drone
        cflib.crtp.init_drivers()
        self.get_logger().info(f"Connexion à {URI}...")
        self.scf = SyncCrazyflie(URI)
        self.scf.open_link()
        self.get_logger().info("✅ Connecté ! Configuration du suivi de cible...")

        # Configuration de la télémétrie ROS 2
        self.log_conf = LogConfig(name='Position', period_in_ms=100)
        self.log_conf.add_variable('stateEstimate.x', 'float')
        self.log_conf.add_variable('stateEstimate.y', 'float')
        self.log_conf.add_variable('stateEstimate.z', 'float')
        self.scf.cf.log.add_config(self.log_conf)
        self.log_conf.data_received_cb.add_callback(self._publish_ros2_data)
        self.log_conf.start()

        # Démarrage du thread de vol
        self.flight_thread = threading.Thread(target=self.flight_sequence)
        self.flight_thread.start()

    def turtle_cb(self, msg):
        # C'est ici qu'on gère le décalage (Offset) !
        # Le (0,0) du TurtleBot correspond au (1,1) du LPS
        tb_x_odom = msg.pose.pose.position.x
        tb_y_odom = msg.pose.pose.position.y
        
        self.target_x = tb_x_odom + 1.0
        self.target_y = tb_y_odom + 1.0

    def flight_sequence(self):
        self.get_logger().info("⏳ Stabilisation (3 secondes)...")
        time.sleep(3)
        
        self.get_logger().info("🚀 DECOLLAGE !")
        with PositionHlCommander(self.scf, default_height=1.0) as pc:
            time.sleep(2)
            
            self.get_logger().info("🎯 Mode Suivi de Cible ACTIVÉ !")
            
            # Boucle infinie : le drone met à jour sa cible 10 fois par seconde
            while rclpy.ok():
                # --- GEOFENCING (Sécurité vitale) ---
                # On force la cible à rester dans la zone de l'arène LPS (ex: entre 0.0 et 3.0 mètres)
                # Si le TurtleBot sort de la zone, le drone s'arrête à la frontière invisible.
                safe_x = max(0.0, min(3.0, self.target_x))
                safe_y = max(0.0, min(3.0, self.target_y))
                safe_z = 1.0 # Altitude constante
                
                # On envoie l'ordre au drone
                self.scf.cf.commander.send_position_setpoint(safe_x, safe_y, safe_z, 0)
                
                # Petite pause pour ne pas inonder l'antenne radio
                time.sleep(0.1) 
                
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
    node = RealLpsFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Arrêt d'urgence ou fin de mission. Atterrissage...")
    
    node.scf.close_link()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
