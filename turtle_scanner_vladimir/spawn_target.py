#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
import random

class SpawnTargetNode(Node):
   def __init__(self):
       super().__init__("spawn_target_node")

       self.get_logger().info("SpawnTargetNode démarré")

       # Création du client de service
       self.spawn_client = self.create_client(Spawn, '/spawn')

       # Attente que le service soit disponible
       while not self.spawn_client.wait_for_service(timeout_sec=1.0):
           self.get_logger().info('En attente du service /spawn...')

       # On spawn la tortue cible une seule fois
       self.spawn_random_target()

   def spawn_random_target(self):
       req = Spawn.Request()
       req.x = random.uniform(1.0, 10.0)
       req.y = random.uniform(1.0, 10.0)
       req.theta = 0.0
       req.name = 'turtle_target'

       future = self.spawn_client.call_async(req)

       # On attend la réponse (comme dans le rappel du TP)
       rclpy.spin_until_future_complete(self, future)

       if future.result() is not None:
           self.get_logger().info(
               f'Tortue cible spawnée à ({req.x:.2f}, {req.y:.2f})'
           )
       else:
           self.get_logger().error('Échec du spawn de la cible')

def main(args=None):
   rclpy.init(args=args)
   node = SpawnTargetNode()
   node.destroy_node()
   rclpy.shutdown()

if __name__ == '__main__':
   main()
