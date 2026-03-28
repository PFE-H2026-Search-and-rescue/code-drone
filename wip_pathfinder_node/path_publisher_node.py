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
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, LivelinessPolicy, HistoryPolicy


class Path_Publisher_Node(Node):
    web_viewer = None

    def __init__(self):
        return;
        super().__init__('path_publisher_node')
        qos_profile = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        liveliness=LivelinessPolicy.AUTOMATIC,
        depth=5,
        history=HistoryPolicy.KEEP_LAST
                                                                                    )
        
        #TODO : Les bons params
        self.publisher_ = self.create_publisher(String, 'le_topic_du_robot', 10, qos_profile=qos_profile)


        self.get_logger().info('robot path node started')