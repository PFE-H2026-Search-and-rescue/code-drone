import json
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from flask import request
import numpy as np
from scipy.spatial.transform import Rotation
from spatial_transforms import add_to_matrix, convert_from_object_to_vector3, get_3d_transform

from apscheduler.schedulers.background import BackgroundScheduler


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

        self.offset = {
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
            self.path.pop(0)
            path_vector = convert_from_object_to_vector3(4, self.path[0]) 
            new_path_vector = self.conversion_matrix_to_robot @ path_vector
            
            print(new_path_vector)
            self.path_publish_node.send_goal(new_path_vector[0], new_path_vector[1])
            
            self.path_scheduler = BackgroundScheduler()
            self.path_scheduler.add_job(func=self.update_path, trigger="interval", seconds=10)
            self.path_scheduler.start()

            #todo : activer un callback chaque x secondes qui détecte si un next élément de l'array est plus proche
            #et le choisi pour skip. 
        
        
        print(str(self.path), flush=True)

    def update_path(self):
        current_distance = abs(self.path[0]["x"] - self.robot_true_position["x"]) + abs(self.path[0]["z"] - self.robot_true_position["z"]) 
        index = len(self.path) - 1

        if(len(self.path) <= 0):
            self.path_scheduler.shutdown()
            return

        for point in reversed(self.path):
            if(index == len(self.path) - 1):
                continue

            new_distance = abs(point["x"] - self.robot_true_position["x"]) + abs(point["z"] - self.robot_true_position["z"])
            
            if(new_distance < current_distance):
                del self.path[0:index]
                self.cancel_path()
                path_vector = convert_from_object_to_vector3(4, self.path[0]) 
                new_path_vector = self.conversion_matrix_to_robot @ path_vector
                
                print(new_path_vector)
                self.path_publish_node.send_goal(new_path_vector[0], new_path_vector[1])
                break
            index -= 1

    def cancel_path(self):
        self.path_publish_node.cancel_goal()

    def add_calibration_point(self): 
        try:
            _ = self.drone_coordinate_system.shape
            self.drone_coordinate_system = add_to_matrix(self.robot_tag_position,self.drone_coordinate_system)
            print("Success", flush=True)
        except Exception as e:
            print(e)
            self.drone_coordinate_system = np.array([self.robot_tag_position["x"], self.robot_tag_position["y"], self.robot_tag_position["z"]])
        
        try:
            _ = self.robot_coordinate_system.shape
            self.robot_coordinate_system = add_to_matrix(self.robot_local_position,self.robot_coordinate_system)
        except:
            self.robot_coordinate_system = np.array([self.robot_local_position["x"], self.robot_local_position["y"], self.robot_local_position["z"]])

        return "true"

    def generate_transform_matrix(self):
        print(self.drone_coordinate_system, flush=True)
        print(self.robot_coordinate_system, flush=True)
        self.conversion_matrix_from_robot = get_3d_transform(self.robot_coordinate_system, self.drone_coordinate_system)
        self.conversion_matrix_to_robot = get_3d_transform(self.drone_coordinate_system, self.robot_coordinate_system)
        print(self.conversion_matrix_from_robot, flush=True)
        print(self.conversion_matrix_to_robot, flush=True)
        return "true";


    def tag_detection_callback(self, value):

        if(self.drone_position["qx"] == 0 and self.drone_position["qy"] == 0 and
            self.drone_position["qz"] == 0 and self.drone_position["qw"] == 0):
            return
        
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

        try:
            _ = self.conversion_matrix_from_robot.shape
            if(abs(self.robot_tag_position["x"] - self.robot_true_position["x"]) + abs(self.robot_tag_position["y"] - self.robot_true_position["y"]) + abs(self.robot_tag_position["z"] - self.robot_true_position["z"]) > 2):
                self.offset["x"] = self.robot_tag_position["x"] - self.robot_true_position["x"]
                self.offset["y"] = self.robot_tag_position["y"] - self.robot_true_position["y"]
                self.offset["z"] = self.robot_tag_position["z"] - self.robot_true_position["z"]
                
        except Exception as e:
            if(type(e) is not AttributeError):
                print(e, flush=True)
            if(self.robot_local_position["x"] != 0):
                self.add_calibration_point()
        


        self.convert_robot_position(True);

    def drone_qvio_callback(self, value : Pose):
        self.drone_position["x"] = value.position.x
        self.drone_position["y"] = value.position.y
        self.drone_position["z"] = value.position.z

        self.drone_position["qx"] = value.orientation.x
        self.drone_position["qy"] = value.orientation.y
        self.drone_position["qz"] = value.orientation.z
        self.drone_position["qw"] = value.orientation.w


    def robot_local_position_callback(self, value):
        self.robot_local_position["x"] = value.x
        self.robot_local_position["y"] = value.y
        self.robot_local_position["z"] = value.z
        self.convert_robot_position(False)
        # self.robot_local_position["qx"] = value.orientation.x
        # self.robot_local_position["qy"] = value.orientation.y
        # self.robot_local_position["qz"] = value.orientation.z
        # self.robot_local_position["qw"] = value.orientation.w

    def convert_robot_position(self, from_tag):
        try:
            _ = self.conversion_matrix_from_robot.shape
            if(from_tag == False):
                new_position = self.conversion_matrix_from_robot @ convert_from_object_to_vector3(4, self.robot_local_position)
                self.robot_true_position["x"] = new_position[0] + self.offset["x"]
                self.robot_true_position["y"] = new_position[1] + self.offset["y"]
                self.robot_true_position["z"] = new_position[2] + self.offset["z"]
                # print(str(self.robot_true_position), flush=True)

        except Exception as e:
            if(type(e) is not AttributeError):
                print(e, flush=True)
        
            
            self.robot_true_position["x"] = self.robot_tag_position["x"]
            self.robot_true_position["y"] = self.robot_tag_position["y"]
            self.robot_true_position["z"] = self.robot_tag_position["z"]