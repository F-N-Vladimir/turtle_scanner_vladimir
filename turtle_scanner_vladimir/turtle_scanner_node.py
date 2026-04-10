#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose

class TurtleScannerNode(Node):
   def __init__(self):
       super().__init__("turtle_scanner_node")

       self.get_logger().info("TurtleScannerNode démarré - Abonnements aux poses activés")

       # Stockage des positions
       self.pose_scanner = None
       self.pose_target = None

       # Abonnements (subscribers)
       self.create_subscription(Pose, '/turtle1/pose',
                               self.pose_scanner_callback, 10)

       self.create_subscription(Pose, '/turtle_target/pose',
                               self.pose_target_callback, 10)

   def pose_scanner_callback(self, msg):
       self.pose_scanner = msg
       # self.get_logger().info(f"Scanner pose mise à jour: x={msg.x:.2f}, y={msg.y:.2f}")

   def pose_target_callback(self, msg):
       self.pose_target = msg
       # self.get_logger().info(f"Cible pose mise à jour: x={msg.x:.2f}, y={msg.y:.2f}")

def main(args=None):
   rclpy.init(args=args)
   node = TurtleScannerNode()
   rclpy.spin(node)          
   node.destroy_node()
   rclpy.shutdown()

if __name__ == '__main__':
   main()
