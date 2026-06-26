import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray, Pose


class FlockViewer(Node):
    def __init__(self):
        super().__init__('flock_viewer')

        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('frame_id', 'world')
        self.declare_parameter('discover_period', 1.0)
        rate            = self.get_parameter('rate_hz').value
        self.frame_id   = self.get_parameter('frame_id').value
        discover_period = self.get_parameter('discover_period').value

        self.latest = {}        # boid_id -> latest Odometry msg
        self.subs = {}          # boid_id -> Subscription object

        self.pose_pub = self.create_publisher(PoseArray, '/boids/poses', 10)

        self.timer = self.create_timer(1.0 / rate, self.publish)

        # Run discovery once immediately, then on a timer
        self.discover()
        self.discover_timer = self.create_timer(discover_period, self.discover)

        self.get_logger().info(
            f'flock_viewer running at {rate:.0f} Hz, discovering every {discover_period}s'
        )

    def discover(self):
        """Scan for /boid_*/odom topics; subscribe to any new ones."""
        all_topics = self.get_topic_names_and_types()

        for topic_name, types in all_topics:
            if 'nav_msgs/msg/Odometry' not in types:
                continue

            if not topic_name.startswith('/boid_') or not topic_name.endswith('/odom'):
                continue

            # Extract the boid id from the topic name: '/boid_7/odom' -> 7
            try:
                id_str = topic_name.split('/')[1].replace('boid_', '')
                boid_id = int(id_str)
            except (ValueError, IndexError):
                continue

            if boid_id in self.subs:
                continue

            # New boid! Subscribe.
            self.subs[boid_id] = self.make_subscription(boid_id)
            self.get_logger().info(
                f'discovered boid {boid_id}, now watching {len(self.subs)}'
            )

    def make_subscription(self, boid_id):
        """Subscribe to one boid's odom topic, stashing latest msgs by id."""
        topic = f'/boid_{boid_id}/odom'

        def callback(msg):
            self.latest[boid_id] = msg

        return self.create_subscription(Odometry, topic, callback, 10)

    def publish(self):
        msg = PoseArray()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()

        for boid_id in sorted(self.latest):
            odom = self.latest[boid_id]
            pose = Pose()
            pose.position.x = odom.pose.pose.position.x
            pose.position.y = odom.pose.pose.position.y
            pose.position.z = odom.pose.pose.position.z
            pose.orientation = odom.pose.pose.orientation
            msg.poses.append(pose)

        self.pose_pub.publish(msg)


def main():
    rclpy.init()
    node = FlockViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()