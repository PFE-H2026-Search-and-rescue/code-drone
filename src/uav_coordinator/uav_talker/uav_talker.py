# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use it except in compliance with the License.
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
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from std_msgs.msg import String


class UavTalkListenerNode(Node):
    """
    DDS listener that subscribes to the talk topic and processes incoming messages.
    Uses the same DDS layer as ROS 2 (e.g. Cyclone DDS / Fast DDS).

    To connect to a remote robot, run via the launch file so DDS is configured with
    the robot as a discovery peer:
      ros2 launch uav_coordinator talk_listener_remote.launch.py robot_ip:=<robot_ip>
    Ensure the robot uses the same ROS_DOMAIN_ID and runs a publisher on the 'talk' topic.
    """

    TALK_TOPIC = "talk"

    def __init__(self):
        super().__init__("uav_talk_listener")
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._sub = self.create_subscription(
            String,
            self.TALK_TOPIC,
            self._on_talk,
            qos_profile=qos,
        )
        self.get_logger().info(f"Listening on topic '{self.TALK_TOPIC}'")

    def _on_talk(self, msg: String) -> None:
        self.get_logger().info(f"talk: {msg.data}")


def main(args=None):
    rclpy.init(args=args)
    node = UavTalkListenerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
