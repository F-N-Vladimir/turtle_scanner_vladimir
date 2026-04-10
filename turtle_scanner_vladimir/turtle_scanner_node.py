#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from turtle_interfaces.srv import ResetMission
from turtlesim.srv import Kill, Spawn
import math
import random

class TurtleScannerNode(Node):
   def __init__(self):
       super().__init__("turtle_scanner_node")

       self.get_logger().info("TurtleScannerNode démarré - Service ResetMission activé")

       self.nb_lignes = 5
       self.y_start = 1.0
       self.y_step = 2.0
       self.x_min = 1.0
       self.x_max = 10.0
       self.linear_speed_max = 2.0
       self.waypoint_tolerance = 0.3
       self.Kp_ang = 2.5
       self.Kp_lin = 1.0
       self.detection_radius = 1.5

       self.waypoints = []
       for i in range(self.nb_lignes):
           y = self.y_start + i * self.y_step
           x = self.x_max if i % 2 == 0 else self.x_min
           self.waypoints.append((x, y))

       self.current_waypoint = 0
       self.scanning = True
       self.target_detected = False

       self.pose_scanner = None
       self.pose_target = None

       self.vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
       self.detected_pub = self.create_publisher(Bool, '/target_detected', 10)
      
       self.create_subscription(Pose, '/turtle1/pose', self.pose_scanner_callback, 10)
       self.create_subscription(Pose, '/turtle_target/pose', self.pose_target_callback, 10)

      
       self.reset_service = self.create_service(
           ResetMission, '/reset_mission', self.reset_mission_callback)

     
       self.timer = self.create_timer(0.05, self.scan_step)

   def pose_scanner_callback(self, msg): self.pose_scanner = msg
   def pose_target_callback(self, msg): self.pose_target = msg

   def compute_angle(self, A, B):
       return math.atan2(B[1] - A[1], B[0] - A[0])

   def compute_distance(self, A, B):
       return math.sqrt((B[0] - A[0])**2 + (B[1] - A[1])**2)

   def publish_zero_vel(self):
       twist = Twist()
       self.vel_pub.publish(twist)

  
   def reset_mission_callback(self, request, response):
       self.get_logger().info("🔄 Réinitialisation de la mission demandée...")

      
       kill_cli = self.create_client(Kill, '/kill')
       kill_req = Kill.Request()
       kill_req.name = 'turtle_target'
       future_kill = kill_cli.call_async(kill_req)
       rclpy.spin_until_future_complete(self, future_kill)

       
       spawn_cli = self.create_client(Spawn, '/spawn')
       spawn_req = Spawn.Request()
       spawn_req.name = 'turtle_target'
       spawn_req.theta = 0.0

       if request.random_target:
           spawn_req.x = random.uniform(1.0, 10.0)
           spawn_req.y = random.uniform(1.0, 10.0)
       else:
           spawn_req.x = request.target_x
           spawn_req.y = request.target_y

       future_spawn = spawn_cli.call_async(spawn_req)
       rclpy.spin_until_future_complete(self, future_spawn)

       
       self.current_waypoint = 0
       self.scanning = True
       self.target_detected = False

       response.success = True
       response.message = "Mission réinitialisée avec succès"
       self.get_logger().info(response.message)
       return response

   
   def scan_step(self):
       if not self.scanning or self.pose_scanner is None:
           self.publish_zero_vel()
           return

       
       if self.pose_target is not None:
           A = (self.pose_scanner.x, self.pose_scanner.y)
           B = (self.pose_target.x, self.pose_target.y)
           dist_target = self.compute_distance(A, B)
           msg = Bool()
           if dist_target < self.detection_radius:
               if not self.target_detected:
                   self.target_detected = True
                   self.scanning = False
                   self.publish_zero_vel()
                   self.get_logger().info(f' Cible détectée à ({self.pose_target.x:.2f}, {self.pose_target.y:.2f}) !')
               msg.data = True
           else:
               self.target_detected = False
               msg.data = False
           self.detected_pub.publish(msg)

       if not self.scanning:
           return

       
       target = self.waypoints[self.current_waypoint]
       A = (self.pose_scanner.x, self.pose_scanner.y)
       dist = self.compute_distance(A, target)

       if dist < self.waypoint_tolerance:
           self.current_waypoint += 1
           if self.current_waypoint >= len(self.waypoints):
               self.get_logger().info(" Balayage terminé")
               self.scanning = False
               self.publish_zero_vel()
           return

       theta_desired = self.compute_angle(A, target)
       theta = self.pose_scanner.theta
       e = math.atan(math.tan((theta_desired - theta) / 2))

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
