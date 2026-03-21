import json
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from flask import request
import numpy as np
from scipy.spatial.transform import Rotation
from spatial_transforms import add_to_matrix, convert_from_object_to_vector3, get_3d_transform

class Mapper_Server():
    def __init__(self, path_publish_node):
        self.robot_tag_position = {
            "x" : 0,
            "y" : 0,
            "z" : 0
        }

        self.drone_position = {
            "x" : 0,
            "y" : 0,
            "z" : 0,
            "qx" : 0,
            "qy" : 0,
            "qz" : 0,
            "qw" : 0
        }

        self.robot_local_position = {
            "x" : 0,
            "y" : 0,
            "z" : 0,
            "qx" : 0,
            "qy" : 0,
            "qz" : 0,
            "qw" : 0
        }

        self.robot_true_position = {
            "x" : 0,
            "y" : 0,
            "z" : 0
        }

        self.conversion_matrix_to_robot = None
        self.conversion_matrix_from_robot = None
        self.drone_coordinate_system = None
        self.robot_coordinate_system = None
        self.path_publish_node = path_publish_node
        
    def get_robot_position(self):
        return self.robot_true_position
    
    def send_path(self):
        self.path = request.json
        if(len(self.path) > 1):
            path_vector = convert_from_object_to_vector3(4, self.path[1]) 
            new_path_vector = self.conversion_matrix_to_robot @ path_vector
            
            #TODO : Convert dans le bon data type
            self.path_publish_node.publisher.publish(new_path_vector)
        
        
        print(str(self.path), flush=True)

    def add_calibration_point(self): 
        if(self.drone_coordinate_system == None):
            self.drone_coordinate_system = np.array([self.robot_tag_position["x"]], [self.robot_tag_position["y"]], [self.robot_tag_position["z"]])
        else:
            add_to_matrix(self.robot_tag_position,self.drone_coordinate_system)
        
        if(self.robot_coordinate_system == None):
            self.robot_coordinate_system = np.array([self.robot_local_position["x"]], [self.robot_local_position["y"]], [self.robot_local_position["z"]])
        else:
            add_to_matrix(self.robot_tag_position,self.robot_coordinate_system)

    def generate_transform_matrix(self):
        self.conversion_matrix_from_robot = get_3d_transform(self.robot_coordinate_system, self.drone_coordinate_system)
        self.conversion_matrix_to_robot = get_3d_transform(self.drone_coordinate_system, self.robot_coordinate_system)


    def tag_detection_callback(self, value : Point):

        q = np.array([self.drone_position["qx"],self.drone_position["qy"], self.drone_position["qz"],self.drone_position["qw"]])
        rotation = Rotation.from_quat(q)
        
        vectors = np.array([[
            value.z,
            value.x,
            value.y
        ]])

        rotated_vectors = rotation.apply(vectors)

        vectors = np.array([[
            self.drone_position["x"] + rotated_vectors[0][0] ,
            self.drone_position["y"] + rotated_vectors[0][1] ,
            self.drone_position["z"] + rotated_vectors[0][2] 
        ]])
        
        r = Rotation.from_rotvec(np.pi/2  * np.array([1, 0, 0]))
        rotated_vectors = r.apply(vectors)

        self.robot_tag_position["x"] = rotated_vectors[0][0]
        self.robot_tag_position["y"] = rotated_vectors[0][1]
        self.robot_tag_position["z"] = rotated_vectors[0][2]

        print("new coordinate", flush=True)
        print(str(self.robot_tag_position), flush=True)
        self.convert_robot_position(True);

    def drone_qvio_callback(self, value : Pose):
        self.drone_position["x"] = value.position.x
        self.drone_position["y"] = value.position.y
        self.drone_position["z"] = value.position.z

        self.drone_position["qx"] = value.orientation.x
        self.drone_position["qy"] = value.orientation.y
        self.drone_position["qz"] = value.orientation.z
        self.drone_position["qw"] = value.orientation.w

    def robot_local_position_callback(self, value : Pose):
        self.robot_local_position["x"] = value.position.x
        self.robot_local_position["y"] = value.position.y
        self.robot_local_position["z"] = value.position.z

        self.robot_local_position["qx"] = value.orientation.x
        self.robot_local_position["qy"] = value.orientation.y
        self.robot_local_position["qz"] = value.orientation.z
        self.robot_local_position["qw"] = value.orientation.w

    def convert_robot_position(self, from_tag):
        if(self.conversion_matrix == None):
            self.robot_true_position["x"] = self.robot_tag_position["x"]
            self.robot_true_position["y"] = self.robot_tag_position["y"]
            self.robot_true_position["z"] = self.robot_tag_position["z"]
        
        if(from_tag == False and self.conversion_matrix != None):
            new_position = self.conversion_matrix_from_robot @ convert_from_object_to_vector3(4, self.robot_local_position)
            self.robot_true_position["x"] = new_position[0]
            self.robot_true_position["y"] = new_position[1]
            self.robot_true_position["z"] = new_position[2]