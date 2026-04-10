#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import math

class TurtleScannerNode(Node):
   def __init__(self):
       super().__init__("turtle_scanner_node")

       self.get_logger().info("✅ TurtleScannerNode démarré - Balayage serpentin activé")

       # === PARAMÈTRES (facile à modifier) ===
       self.nb_lignes = 5
       self.y_start = 1.0
       self.y_step = 2.0
       self.x_min = 1.0
       self.x_max = 10.0
       self.linear_speed_max = 2.0
       self.waypoint_tolerance = 0.3
       self.Kp_ang = 2.5      # Valeur que tu noteras dans le README
       self.Kp_lin = 1.0      # Valeur que tu noteras dans le README

       # Génération des waypoints en serpentin (comme demandé dans le TP)
       self.waypoints = []
       for i in range(self.nb_lignes):
           y = self.y_start + i * self.y_step
           x = self.x_max if i % 2 == 0 else self.x_min
           self.waypoints.append((x, y))

       self.current_waypoint = 0
       self.scanning = True

       # Positions des tortues
       self.pose_scanner = None
       self.pose_target = None

       # Publisher pour la vitesse
       self.vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

       # Abonnements (Partie 2)
       self.create_subscription(Pose, '/turtle1/pose',
                               self.pose_scanner_callback, 10)
       self.create_subscription(Pose, '/turtle_target/pose',
                               self.pose_target_callback, 10)

       # Timer 20 Hz (comme suggéré dans le TP)
       self.timer = self.create_timer(0.05, self.scan_step)

   def pose_scanner_callback(self, msg):
       self.pose_scanner = msg

   def pose_target_callback(self, msg):
       self.pose_target = msg

   def compute_angle(self, A, B):
       """Calcule l'angle désiré (comme demandé dans le TP)"""
       return math.atan2(B[1] - A[1], B[0] - A[0])

   def compute_distance(self, A, B):
       """Distance euclidienne (comme demandé dans le TP)"""
       return math.sqrt((B[0] - A[0])**2 + (B[1] - A[1])**2)

   def publish_zero_vel(self):
       """Arrête la tortue"""
       twist = Twist()
       self.vel_pub.publish(twist)

   def scan_step(self):
       if not self.scanning or self.pose_scanner is None:
           self.publish_zero_vel()
           return

       # Waypoint actuel
       target = self.waypoints[self.current_waypoint]
       A = (self.pose_scanner.x, self.pose_scanner.y)

       dist = self.compute_distance(A, target)

       # Si on est assez proche du waypoint
       if dist < self.waypoint_tolerance:
           self.current_waypoint += 1
           if self.current_waypoint >= len(self.waypoints):
               self.get_logger().info("✅ Balayage terminé")
               self.scanning = False
               self.publish_zero_vel()
           return

       # Commande de mouvement (régulation proportionnelle)
       theta_desired = self.compute_angle(A, target)
       theta = self.pose_scanner.theta
       e = math.atan(math.tan((theta_desired - theta) / 2))   # Formule exacte demandée dans le TP

       u = self.Kp_ang * e
       v = min(self.Kp_lin * dist, self.linear_speed_max)

       twist = Twist()
       twist.linear.x = v
       twist.angular.z = u
       self.vel_pub.publish(twist)

def main(args=None):
   rclpy.init(args=args)
   node = TurtleScannerNode()
   rclpy.spin(node)
   node.destroy_node()
   rclpy.shutdown()

if __name__ == '__main__':
   main()
