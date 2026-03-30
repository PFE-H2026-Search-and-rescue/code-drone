from flask import Flask
from flask_cors import CORS, cross_origin

import threading
import rclpy
from mapper_server import Mapper_Server
from tag_detection_node import Tag_Detection_Node
from robot_position_node import Robot_Position_Node
from qvio_node import Qvio_Node
from send_path_client import SendPathClient

app = Flask(__name__)
cors = CORS(app) # allow CORS for all domains on all routes.
app.config['CORS_HEADERS'] = 'Content-Type'



if __name__ == '__main__':
    rclpy.init()

    #Publishers 
    send_path_client = SendPathClient()

    
    mapper_server = Mapper_Server(send_path_client);

    #ROS2
    
    qvio_node = Qvio_Node()
    tag_detection_node = Tag_Detection_Node()
    robot_position_node = Robot_Position_Node()
    
    tag_detection_node.set_backend_server(mapper_server)
    qvio_node.set_backend_server(mapper_server)
    robot_position_node.set_backend_server(mapper_server)
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(tag_detection_node)
    executor.add_node(qvio_node)
    executor.add_node(robot_position_node)
    # executor.add_node(path_publisher_node)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()


    app.add_url_rule('/robot_position', 'robot_position', mapper_server.get_robot_position)
    app.add_url_rule('/send_path', 'send_path', mapper_server.send_path, methods=["POST"])
    app.add_url_rule('/add_calibration_point', 'add_calibration_point', mapper_server.add_calibration_point, methods=["POST"])
    app.add_url_rule('/calibrate', 'calibrate', mapper_server.generate_transform_matrix, methods=["POST"])

    app.run(host='0.0.0.0')
    
    tag_detection_node.destroy_node()
    rclpy.shutdown()

    






