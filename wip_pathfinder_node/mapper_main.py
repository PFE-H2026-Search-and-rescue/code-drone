from flask import Flask
from flask_cors import CORS, cross_origin

import threading
import rclpy
from mapper_server import Mapper_Server
from tag_detection_node import Tag_Detection_Node

app = Flask(__name__)
cors = CORS(app) # allow CORS for all domains on all routes.
app.config['CORS_HEADERS'] = 'Content-Type'



if __name__ == '__main__':
    mapper_server = Mapper_Server();

    #ROS2
    rclpy.init()
    tag_detection_node = Tag_Detection_Node()
    tag_detection_node.set_backend_server(mapper_server)
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(tag_detection_node)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()


    app.add_url_rule('/robot_position', 'robot_position', mapper_server.get_robot_position)
    app.add_url_rule('/send_path', 'send_path', mapper_server.send_path, methods=["POST"])
    app.run(host='0.0.0.0')
    
    tag_detection_node.destroy_node()
    rclpy.shutdown()

    






