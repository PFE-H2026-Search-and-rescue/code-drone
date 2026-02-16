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

from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, LivelinessPolicy, HistoryPolicy

class Camera_Data_Node(Node):

    def __init__(self):
        super().__init__('camera-data-node')
        qos_profile = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        liveliness=LivelinessPolicy.AUTOMATIC,
        depth=5,
        history=HistoryPolicy.KEEP_LAST
                                                                                    )
        self.subscription = self.create_subscription(
            Image,
            '/mapper',
            self.qvio_callback,
            qos_profile=qos_profile)

        self.get_logger().info('camera node started')

    def qvio_callback(self, msg : Image):
        self.get_logger().info(str(msg.data))



def main(args=None):
    rclpy.init(args=args)

    camera_node = Camera_Data_Node()

    rclpy.spin(camera_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    camera_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
