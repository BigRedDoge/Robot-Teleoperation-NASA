import sqlite3
import os
import cv2
import numpy as np
import rospy
from ultralytics import YOLO
import torch
from camera.msg import Image
from ai_data.srv import GetAIData

# import the message types

class AIDBService:
    def __init__(self):
        self.connection = sqlite3.connect('ai_db.db')
        self.cursor = self.connection.cursor()
        self.db_init()

        # load the yolo model
        model_path = 'yolov5s.pt'
        self.yolo = YOLO(model_path)
        # load the model to the gpu if available
        if torch.cuda.is_available():
            self.yolo.cuda()

        self.frame = None

        # initialize the node
        rospy.init_node('ai_db_service', anonymous=True)
        # subscribe to the camera data
        rospy.Subscriber('camera_data', Image, self.camera_callback)
        # create the ai db service
        rospy.Service('get_ai_data', GetAIData, self.get_ai_data)
        rospy.spin()
        
    def db_init(self):
        table_name = 'classification'
        # check if table exists
        exists = self.cursor.execute(f"SELECT name FROM sqlite_master WHERE type='table' AND name='{table_name}';")
        if not exists.fetchone():
            rospy.loginfo("Creating table for classification")
            # create the table
            self.cursor.execute(f'''
                CREATE TABLE IF NOT EXISTS {table_name} (
                    id INTEGER PRIMARY KEY,
                    class VARCHAR(255),
                    instructions TEXT,
                    3d_model VARCHAR(255)
                )
            ''')
    """
    Set self.frame to the image data published from the camera
    """
    def camera_callback(self, msg):
        """
        Callback function for the camera data
        """
        # convert the image to a numpy array
        image = np.asarray(bytearray(msg.data), dtype=np.uint8)
        # decode the image
        self.frame = cv2.imdecode(image, cv2.IMREAD_COLOR)

    """
    Respond to get_ai_data srv request with the class name, instructions, and 3d model for the object in the image
    """
    def get_ai_data(self, req):
        """
        Service to get ai data
        """
        # get the image from the camera
        frame = self.frame
        # run the yolo model on the image
        results = self.yolo(frame)
        # get the class of the object
        class_name = results.names[0]
        # get the instructions for the class
        instructions, model = self.get_data(class_name)

        # create the response srv message
        response = GetAIData()
        response.class_name = class_name
        response.instructions = instructions
        response.model = model

        return response
    
    """
    Get the data for the class from the database
    """
    def get_data(self, class_name):
        """
        Get the data for the class
        """
        # get the data from the database
        self.cursor.execute(f"SELECT * FROM classification WHERE class='{class_name}'")
        data = self.cursor.fetchone()
        if data:
            return data[1], data[2]
        else:
            return None, None


if __name__ == '__main__':
    AIDBService()