import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry  # <-- NOUVEAU : Pour comprendre la position du TurtleBot
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import threading

# Listes pour stocker les historiques des deux robots
x_drone, y_drone = [], []
x_turtle, y_turtle = [], []

class LivePlotNode(Node):
    def __init__(self):
        super().__init__('live_plot_node')
        
        # 1. Abonnement au Drone (LPS)
        self.sub_drone = self.create_subscription(
            PoseStamped, '/crazyflie/lps_pose', self.drone_callback, 10)
            
        # 2. Abonnement au TurtleBot (Odométrie)
        self.sub_turtle = self.create_subscription(
            Odometry, '/tb4/odom', self.turtle_callback, 10)
            
        self.get_logger().info("Écoute du Drone et du TurtleBot démarrée !")

    def drone_callback(self, msg):
        x_drone.append(msg.pose.position.x)
        y_drone.append(msg.pose.position.y)
        if len(x_drone) > 5000:  # On garde les 300 derniers points (une plus longue trace)
            x_drone.pop(0)
            y_drone.pop(0)

    def turtle_callback(self, msg):
        x_turtle.append(msg.pose.pose.position.x)
        y_turtle.append(msg.pose.pose.position.y)
        if len(x_turtle) > 300:
            x_turtle.pop(0)
            y_turtle.pop(0)

def ros_spin_thread(node):
    rclpy.spin(node)

def update_plot(frame):
    plt.cla() # Effacer l'image précédente
    
    # 1. Dessiner le Drone en BLEU
    if len(x_drone) > 0:
        plt.plot(x_drone, y_drone, 'b-', alpha=0.5, label='Trace Drone')
        plt.plot(x_drone[-1], y_drone[-1], 'bo', markersize=8, label='Drone (Actuel)')
        
    # 2. Dessiner le TurtleBot en VERT
    if len(x_turtle) > 0:
        plt.plot(x_turtle, y_turtle, 'g-', alpha=0.5, label='Trace TurtleBot')
        plt.plot(x_turtle[-1], y_turtle[-1], 'gs', markersize=8, label='TurtleBot (Actuel)')
        
    # Paramètres de la fenêtre
    plt.title("Suivi des Robots en Temps Réel")
    plt.xlabel("X (mètres)")
    plt.ylabel("Y (mètres)")
    plt.grid(True, linestyle='--')
    plt.axis('equal') # Indispensable pour ne pas déformer les trajectoires
    plt.legend(loc='upper right')

def main(args=None):
    rclpy.init(args=args)
    node = LivePlotNode()

    # Lancer ROS 2 en arrière-plan
    thread = threading.Thread(target=ros_spin_thread, args=(node,), daemon=True)
    thread.start()

    # Lancer le graphique
    fig = plt.figure(figsize=(8, 8))
    ani = animation.FuncAnimation(fig, update_plot, interval=100)
    plt.show()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
