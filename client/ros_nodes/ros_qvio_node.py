import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class UAVMapper(Node):
    def __init__(self):
        super().__init__('uav_mapper')
        self.get_logger().info('UAV Mapper initialized')

        # Subscribe to QVIO odometry topic
        self.qvio_subscription = self.create_subscription(
            Odometry,
            '/qvio',
            self.qvio_callback,
            10
        )
        self.get_logger().info('Subscribed to /qvio/odometry')

    def qvio_callback(self, msg):
        """Callback function for QVIO odometry messages"""
        # Extract position
        position = msg.pose.pose.position
        # Extract orientation (quaternion)
        orientation = msg.pose.pose.orientation
        # Extract linear velocity
        linear_vel = msg.twist.twist.linear
        # Extract angular velocity
        angular_vel = msg.twist.twist.angular

        self.get_logger().info(
            f'QVIO - Position: x={position.x:.3f}, y={position.y:.3f}, z={position.z:.3f} | '
            f'Linear Vel: x={linear_vel.x:.3f}, y={linear_vel.y:.3f}, z={linear_vel.z:.3f}'
        )


def main():
    rclpy.init()
    node = UAVMapper()
    rclpy.spin(node)
    rclpy.shutdown()
