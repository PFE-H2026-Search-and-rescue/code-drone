# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, LivelinessPolicy, HistoryPolicy

class QVIO_Node(Node):

    def __init__(self):
        super().__init__('qvio_node')
        qos_profile = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        liveliness=LivelinessPolicy.AUTOMATIC,
        depth=5,
        history=HistoryPolicy.KEEP_LAST
                                                                                    )
        self.subscription = self.create_subscription(
            PoseStamped,
            '/qvio',
            self.qvio_callback,
            qos_profile=qos_profile)

        self.get_logger().info('qvio node started')

    def qvio_callback(self, msg : PoseStamped):
        self.get_logger().info(str(msg.pose.position.x) + "," + str(msg.pose.position.y) + "," + str(msg.pose.position.z))

    #def qvio_odemetry_callback(self, msg):
    #    """Callback function for QVIO odometry messages"""
    #    # Extract position
    #    position = msg.pose.pose.position
    #    # Extract orientation (quaternion)
    #    orientation = msg.pose.pose.orientation
    #    # Extract linear velocity
    #    linear_vel = msg.twist.twist.linear
    #    # Extract angular velocity
    #    angular_vel = msg.twist.twist.angular
    #
    #    self.get_logger().info(
    #        f'QVIO - Position: x={position.x:.3f}, y={position.y:.3f}, z={position.z:.3f} | '
    #        f'Linear Vel: x={linear_vel.x:.3f}, y={linear_vel.y:.3f}, z={linear_vel.z:.3f}'
    #    )

def main(args=None):
    rclpy.init(args=args)

    qvio_node = QVIO_Node()

    rclpy.spin(qvio_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    qvio_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
