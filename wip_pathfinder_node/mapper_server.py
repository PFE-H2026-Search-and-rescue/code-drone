import json
from geometry_msgs.msg import Point
from flask import request

class Mapper_Server():
    def __init__(self):
        self.robot_position = {
            "x" : 0,
            "y" : 0,
            "z" : 0
        }

        
    def get_robot_position(self):
        return self.robot_position
    
    def send_path(self):
        self.path = request.json
        print(str(self.path))
        #TODO : publish la path (ne pas oublier de la mettre dans avec le bon up (faire inverse de detection_callback))

    def tag_detection_callback(self, value : Point):
        self.robot_position["x"] = value.x
        self.robot_position["y"] = value.z
        self.robot_position["z"] = -value.y