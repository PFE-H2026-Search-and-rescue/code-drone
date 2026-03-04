
import threading
import rclpy
from point_cloud_node import Point_Cloud_Node
from web_viewer import Web_Viewer

#Server
web_viewer = Web_Viewer()

#ROS2
rclpy.init()
mapper_node = Point_Cloud_Node()
mapper_node.set_web_viewer(web_viewer)
executor = rclpy.executors.MultiThreadedExecutor()
executor.add_node(mapper_node)
executor_thread = threading.Thread(target=executor.spin, daemon=True)
executor_thread.start()

#Start
print("pretest")
web_viewer.start_server()
print("test")
mapper_node.destroy_node()
rclpy.shutdown()
