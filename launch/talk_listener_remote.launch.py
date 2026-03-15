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

import os
import tempfile

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_with_dds_config(context, *args, **kwargs):
    robot_ip = LaunchConfiguration("robot_ip").perform(context)
    domain_id = LaunchConfiguration("ros_domain_id").perform(context)

    dds_xml = f"""<?xml version="1.0" encoding="UTF-8"?>
<CycloneDDS xmlns="https://cdr.omg.org/dds/cyclonedds/1.0">
  <Domain>
    <Discovery>
      <Peers>
        <Peer address="{robot_ip}"/>
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
"""
    fd, path = tempfile.mkstemp(suffix=".xml", prefix="cyclonedds_")
    try:
        with os.fdopen(fd, "w") as f:
            f.write(dds_xml)
    except Exception:
        os.close(fd)
        raise
    uri = "file://" + path

    # Use system default RMW (e.g. Fast DDS); CYCLONEDDS_URI only applies if Cyclone DDS is active.
    # For Cyclone DDS peer discovery, install ros-humble-rmw-cyclonedds-cpp and set RMW_IMPLEMENTATION=rmw_cyclonedds_cpp.
    return [
        SetEnvironmentVariable(name="CYCLONEDDS_URI", value=uri),
        SetEnvironmentVariable(name="ROS_DOMAIN_ID", value=domain_id),
        Node(
            package="uav_coordinator",
            executable="talk_listener_node",
            name="uav_talk_listener",
            output="screen",
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.1.100",
            description="IP address of the remote robot (DDS discovery peer)",
        ),
        DeclareLaunchArgument(
            "ros_domain_id",
            default_value="0",
            description="ROS_DOMAIN_ID; must match the remote robot",
        ),
        OpaqueFunction(function=_launch_with_dds_config),
    ])
