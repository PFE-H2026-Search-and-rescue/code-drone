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
import time
from os import wait

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, LivelinessPolicy, HistoryPolicy


class Robot_Position_Node(Node):
    web_viewer = None

    def __init__(self):
        super().__init__('robot_position_node')
        qos_profile = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        liveliness=LivelinessPolicy.AUTOMATIC,
        depth=5,
        history=HistoryPolicy.KEEP_LAST
                                                                                    )
        self.subscription = self.create_subscription(
            PoseStamped,
            '/odom_rf2o',
            self.point_cloud_callback,
            qos_profile=qos_profile)

        self.get_logger().info('robot position node started')

    def set_backend_server(self, backend_server):
        self.backend_server = backend_server

    def point_cloud_callback(self, msg : PoseStamped):
        
        self.get_logger().info("callback received")
        self.get_logger().info(str(msg.pose.position))
        self.backend_server.robot_local_position_callback(msg.pose.position)
