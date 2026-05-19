import rclpy
from rclpy.node import Node
import cflib.crtp
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
from cflib.positioning.position_hl_commander import PositionHlCommander
import threading
import time
import csv
import os
from datetime import datetime

URI = 'radio://0/30/2M/E7E7E7E7E7'

class LineFlightNode(Node):
    def __init__(self):
        super().__init__('real_lps_line_node')
        
        # --- 1. CRÉATION DIRECTE DU FICHIER CSV ---
        log_dir = os.path.expanduser('~/ros2_ws/flight_logs')
        os.makedirs(log_dir, exist_ok=True)
        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_filename = os.path.join(log_dir, f'ligne_droite_{timestamp_str}.csv')
        
        self.csv_file = open(self.csv_filename, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['Temps_s', 'Drone_X', 'Drone_Y', 'Drone_Z'])
        self.start_time = time.time()
        self.is_logging = True
        
        # --- 2. CONNEXION AU DRONE ---
        cflib.crtp.init_drivers()
        self.get_logger().info(f"Connexion au drone sur {URI}...")
        self.scf = SyncCrazyflie(URI)
        self.scf.open_link()
        self.get_logger().info("✅ Connecté ! L'EKF embarqué est actif.")

        # --- 3. CONFIGURATION DES CAPTEURS ---
        self.log_conf = LogConfig(name='Position', period_in_ms=100) # 10 Hz
        self.log_conf.add_variable('stateEstimate.x', 'float')
        self.log_conf.add_variable('stateEstimate.y', 'float')
        self.log_conf.add_variable('stateEstimate.z', 'float')
        self.scf.cf.log.add_config(self.log_conf)
        self.log_conf.data_received_cb.add_callback(self._log_data)
        self.log_conf.start()

        self.flight_thread = threading.Thread(target=self.flight_sequence)
        self.flight_thread.start()

    def flight_sequence(self):
        self.get_logger().info("⏳ Stabilisation (2 secondes)...")
        time.sleep(2)
        
        # Décollage RAPIDE (0.5 m/s)
        self.get_logger().info("🚀 Décollage rapide...")
        with PositionHlCommander(self.scf, default_height=1.0, default_velocity=0.5) as pc:
            time.sleep(1)
            
            # Mise en position (X=1.0, Y=1.0)
            self.get_logger().info("🎯 Mise en position de départ en (1.0, 1.0)...")
            pc.go_to(1.0, 1.0, 1.0)
            time.sleep(3) # Pause pour stabiliser avant la ligne
            
            # TRACER LA LIGNE DROITE JUSQU'À (X=3.0, Y=1.0)
            self.get_logger().info("📏 Ligne droite en cours vers (3.0, 1.0)...")
            pc.go_to(3.0, 1.0, 1.0)
            time.sleep(2) # Pause de fin de ligne
            
            self.get_logger().info("🛬 Atterrissage...")

        time.sleep(3)
        self.get_logger().info(f"🏁 Vol terminé. Données sauvées dans : {self.csv_filename}")
        
        # Fermeture propre du fichier CSV
        self.is_logging = False
        self.csv_file.close()
        
    def _log_data(self, timestamp, data, logconf):
        # Cette fonction écrit directement dans le CSV 10 fois par seconde
        if self.is_logging and not self.csv_file.closed:
            t = time.time() - self.start_time
            self.csv_writer.writerow([round(t, 2), data['stateEstimate.x'], data['stateEstimate.y'], data['stateEstimate.z']])

def main(args=None):
    rclpy.init(args=args)
    node = LineFlightNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Arrêt demandé.")
    finally:
        # Sécurité : fermer le fichier même si on force l'arrêt avec Ctrl+C
        if node.is_logging and not node.csv_file.closed:
            node.csv_file.close()
        node.scf.close_link()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
