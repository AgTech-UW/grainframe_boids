import numpy as np
import math
import rclpy
from rclpy.node import Node
from boids.boids_logic import step
from nav_msgs.msg import Odometry

class BoidSingle(Node):
    def __init__(self):
        super().__init__('boid_single')

        self.neighbor_state = {}    # neighbor_id dictionary to hold the state of each neighbor boid
        
        self.declare_parameter('num_boids', 4)
        self.N = self.get_parameter('num_boids').value

        self.declare_parameter('boid_id', 0) # declare the boid_id parameter with a default value of 0
        self.id = self.get_parameter('boid_id').value # get the boid_id parameter from the launch file

        # random initial state
        rng = np.random.default_rng()
        self.x  = float((2 * rng.random() - 0.5) * 110)
        self.y  = float((2 * rng.random() - 0.5) * 70)
        self.vx = float(2 * rng.random() - 0.5)
        self.vy = float(2 * rng.random() - 0.5)

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
        topicName = f'/boid_{self.id}/odom'
        self.pose_pub = self.create_publisher(Odometry, topicName, 10)

        self.subs = []
        for i in range(self.N):
            if i != self.id:
                self.subs.append(self.make_subscription(i))

        # startup message
        self.get_logger().info(f'we got Boid {self.id} running')

    def tick(self):
        xs  = [self.x]
        ys  = [self.y]
        vxs = [self.vx]
        vys = [self.vy]
        for neighbor_id in self.neighbor_state:
            n = self.neighbor_state[neighbor_id]
            xs.append(n['x'])
            ys.append(n['y'])
            vxs.append(n['vx'])
            vys.append(n['vy'])
        
        state = {
            'x':  np.array(xs),
            'y':  np.array(ys),
            'vx': np.array(vxs),
            'vy': np.array(vys),
        }

        state = step(state, self.p) # pass in all the parameters, and the dictionary of positions

        # update the state of x, y, vx, vy for this boid only (index 0, since the first element corresponds to this boid)
        self.x = state['x'][0]
        self.y = state['y'][0]
        self.vx = state['vx'][0]
        self.vy = state['vy'][0]

        # Keep the log line so you can watch the position change
        self.get_logger().info(
            f'boid {self.id} at ({self.x:.1f}, {self.y:.1f}), '
            f'heard from {len(self.neighbor_state)} neighbors'
        )

        # Publish the single boids new state
        self.publish_odom()

    def publish_odom(self):
        msg = Odometry()
        msg.header.frame_id = 'world'
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.pose.pose.position.x = float(self.x)
        msg.pose.pose.position.y = float(self.y)
        msg.pose.pose.position.z = 0.0

        yaw = math.atan2(float(self.vy), float(self.vx))
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        msg.twist.twist.linear.x = float(self.vx)
        msg.twist.twist.linear.y = float(self.vy)

        self.pose_pub.publish(msg)
    
    def make_subscription(self, neighbor_id):
        """Create one subscription that knows which neighbor it's listening to."""
        topic = f'/boid_{neighbor_id}/odom'

        def callback(msg):
            self.neighbor_state[neighbor_id] = {
                'x':  msg.pose.pose.position.x,
                'y':  msg.pose.pose.position.y,
                'vx': msg.twist.twist.linear.x,
                'vy': msg.twist.twist.linear.y,
            }

        sub = self.create_subscription(Odometry, topic, callback, 10)
        return sub


def main():
    rclpy.init()
    node = BoidSingle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
    