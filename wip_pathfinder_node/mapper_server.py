import json
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from flask import request
import numpy as np
from scipy.spatial.transform import Rotation

class Mapper_Server():
    def __init__(self):
        self.robot_position = {
            "x" : 0,
            "y" : 0,
            "z" : 0
        }

        self.drone_position = {
            "x" : 0,
            "y" : 0,
            "z" : 0
        }

        
    def get_robot_position(self):
        return self.robot_position
    
    def send_path(self):
        self.path = request.json
        print(str(self.path), flush=True)
        #TODO : publish la path (ne pas oublier de la mettre dans avec le bon up (faire inverse de detection_callback))

    def tag_detection_callback(self, value : Point):

        q = np.array([self.drone_position["qx"],self.drone_position["qy"], self.drone_position["qz"],self.drone_position["qw"]])
        rotation = Rotation.from_quat(q)
        
        vectors = np.array([[
            value.x,
            value.z,
            value.y
        ]])

        rotated_vectors = rotation.apply(vectors)

        vectors = np.array([[
            self.drone_position["x"] - rotated_vectors[0][0] ,
            self.drone_position["y"] - rotated_vectors[0][1] ,
            self.drone_position["z"] - rotated_vectors[0][2] 
        ]])
        
        # r = Rotation.from_rotvec(np.pi  * np.array([1, 0, 0]))
        # rotated_vectors = r.apply(vectors)

        # self.robot_position["x"] = rotated_vectors[0][0]
        # self.robot_position["y"] = rotated_vectors[0][1]
        # self.robot_position["z"] = rotated_vectors[0][2]

        print("new coordinate", flush=True)
        print(str(self.robot_position), flush=True)

    def drone_qvio_callback(self, value : Pose):
        self.drone_position["x"] = value.position.x
        self.drone_position["y"] = value.position.y
        self.drone_position["z"] = value.position.z

        self.drone_position["qx"] = value.orientation.x
        self.drone_position["qy"] = value.orientation.y
        self.drone_position["qz"] = value.orientation.z
        self.drone_position["qw"] = value.orientation.w