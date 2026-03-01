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

from sensor_msgs.msg import PointCloud2
import point_cloud2 as pc2
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, LivelinessPolicy, HistoryPolicy


class Point_Cloud_Node(Node):
    web_viewer = None
    callbacked = 0

    def __init__(self):
        super().__init__('point_cloud_node')
        qos_profile = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        liveliness=LivelinessPolicy.AUTOMATIC,
        depth=5,
        history=HistoryPolicy.KEEP_LAST
                                                                                    )
        self.subscription = self.create_subscription(
            PointCloud2,
            '/voxl_mapper_aligned_ptcloud',
            self.point_cloud_callback,
            qos_profile=qos_profile)

        self.get_logger().info('qvio node started')

    def set_web_viewer(self, web_viewer):
        self.web_viewer = web_viewer

    def point_cloud_callback(self, msg : PointCloud2):

        if self.callbacked == 1:
            return
        self.get_logger().info("callback received")
        point_cloud = pc2.read_points_numpy(msg)
        self.get_logger().info(str(point_cloud.shape))
        self.get_logger().info(str(point_cloud.dtype))
        if self.web_viewer is not None:
            self.web_viewer.add_points(point_cloud)
        self.callbacked = 1
