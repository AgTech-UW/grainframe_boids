import numpy as np
import math
import rclpy
from rclpy.node import Node
from boids.boids_logic import step
from geometry_msgs.msg import PoseArray, Pose


class BoidsNode(Node):
    def __init__(self):
        super().__init__('boids_node')

        self.N = 12 # Number of Boids

        # random initial state
        rng = np.random.default_rng()
        self.x  = (2 * rng.random(self.N) - 0.5) * 110    # x range scale 
        self.y  = (2 * rng.random(self.N) - 0.5) * 70    # y range scale 
        self.vx = (2 * rng.random(self.N) - 0.5)
        self.vy = (2 * rng.random(self.N) - 0.5)

        # bundle the tuning constants into a dict.
        self.p = {
            'VR': 50,              # visual range 
            'PR': 10,              # protected range
            'CF': 0.0005,          # cohesion factor
            'SF': 0.2,             # separation factor
            'AF': 0.2,             # alignment factor
            'TF': 0.5,             # turning factor
            'max_speed': 3,        # 3
            'min_speed': 1.5,      # 1.5
            'x_safe_min': -100.0,  # field bounds with safety baked in
            'x_safe_max':  100.0,
            'y_safe_min':  -60.0,
            'y_safe_max':   60.0,
        }

        # the boids get updated every 20Hz through the timer
        self.timer = self.create_timer(0.05, self.tick)

        # publisher that broadcasts boid positions for RViz
        self.pose_pub = self.create_publisher(PoseArray, '/boids/poses', 10)

        # startup message
        self.get_logger().info(f'Boids running with {self.N} boids') 

    def tick(self):
            state = {'x': self.x, 'y': self.y, 'vx': self.vx, 'vy': self.vy} #set up dictionary of the current 
            state = step(state, self.p) # pass in all the parameters, and the dictionary of positions
            #update the current position
            self.x  = state['x'] 
            self.y  = state['y']
            self.vx = state['vx']
            self.vy = state['vy']

            # publish the new state for RViz
            self.publish_poses()


    def publish_poses(self):
            msg = PoseArray()
            msg.header.frame_id = 'world'                          # the RViz coordinate frame
            msg.header.stamp = self.get_clock().now().to_msg()     # timestamp = right now

            for i in range(self.N):
                pose = Pose() #use the pose type of message
                pose.position.x = float(self.x[i])
                pose.position.y = float(self.y[i])
                pose.position.z = 0.0

                # get the heading from velocity.
                yaw = math.atan2(float(self.vy[i]), float(self.vx[i]))
                pose.orientation.z = math.sin(yaw / 2.0)
                pose.orientation.w = math.cos(yaw / 2.0)

                msg.poses.append(pose)

            self.pose_pub.publish(msg)

def main():
    rclpy.init()
    node = BoidsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()