#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32 ,String
import mediapipe as mp
import numpy as np
import cv2
from cv_bridge import CvBridge
import ros_numpy
import os
import pathlib
from mediapipe.tasks.python import vision
import json


def get_json_file():
    # Get the current working directory
    current_dir =  os.path.dirname(os.path.abspath(__file__))
    #
    print(current_dir)

    # Loop through all the files in the directory
    for file in os.listdir(current_dir):
        # Check if the file is a JSON file
        if file.endswith('.json'):
            # Return the directory path
            return str(os.path.join(current_dir, file))
            
            break
    else:
        print("No JSON file found in current directory.")

class tiagoDetection:
    def __init__(self):
        self.count = 0
        self.objCenter = (0, 0)
        self.imageWidth = 0
        self.bridge = CvBridge()
        self.imageLabel = ''
        self.models = self.get_models()
        self.objCenter_pub = rospy.Publisher('image_centre', Float32, queue_size=10)
        self.imageWidth_pub = rospy.Publisher('image_width', Float32, queue_size=10)
        self.imageLabel_pub = rospy.Publisher('image_label', String, queue_size=10)
        rospy.Subscriber("/xtion/rgb/image_raw", Image, self.callback, queue_size=1, buff_size=2058)
        
    def get_models(self):
        MODELDIR = pathlib.Path(os.path.join(pathlib.Path(__file__).parent.absolute(),'models')).glob('**/*')
        models = list()
        for i in MODELDIR:
            if i.is_file():
                models.append(i)
        return models
    
    
    
    def visualize(self,image, detection_result) -> np.ndarray:
        MARGIN = 10
        ROW_SIZE = 10
        FONT_SIZE = 1
        FONT_THICKNESS = 1
        TEXT_COLOR = (255, 0, 0)
        centre = (0, 0)
        imageWidth = 0
        imageLabel = ''
        unwanted = ["bed","tv","laptop","train"]
        for detection in detection_result.detections:
            category = detection.categories[0]
            category_name = category.category_name

            if category_name in unwanted:
                break
            bbox = detection.bounding_box
            start_point = bbox.origin_x, bbox.origin_y
            end_point = bbox.origin_x + bbox.width, bbox.origin_y + bbox.height
            centre = bbox.origin_x + (bbox.width * 0.5), bbox.origin_y + (bbox.height * 0.5)
            imageWidth = bbox.width
            cv2.rectangle(image, start_point, end_point, TEXT_COLOR, 3)

            
            imageLabel = category_name

            self.objCenter_pub.publish(centre[0])
            self.imageWidth_pub.publish(imageWidth)
            self.imageLabel_pub.publish(imageLabel)

            probability = round(category.score, 2)
            result_text = category_name + ' (' + str(probability) + ')'
            text_location = (MARGIN + bbox.origin_x,
                            MARGIN + ROW_SIZE + bbox.origin_y)
            cv2.putText(image, result_text, text_location, cv2.FONT_HERSHEY_PLAIN,
                        FONT_SIZE, TEXT_COLOR, FONT_THICKNESS)

        return image , centre , imageWidth , imageLabel


    def callback(self, data):
        if self.count == 0:
                

            #create a counter 
            imageNP = ros_numpy.numpify(data)

            BaseOptions = mp.tasks.BaseOptions
            ObjectDetectorOptions = mp.tasks.vision.ObjectDetectorOptions
            VisionRunningMode = mp.tasks.vision.RunningMode

            efficientdet_lite0= self.models[0]
            #print(f"Model - {efficientdet_lite0}")

            options = ObjectDetectorOptions( 
                base_options=BaseOptions(model_asset_path= efficientdet_lite0),
                max_results=5,
                score_threshold=0.285,
                running_mode=VisionRunningMode.IMAGE)

            detector = vision.ObjectDetector.create_from_options(options)


            # Load the input image.
            image = mp.Image(image_format=mp.ImageFormat.SRGB, data=imageNP)

            #  Detect objects in the input image.
            detection_result = detector.detect(image)

            # Process the detection result. In this case, visualize it.
            image_copy = np.copy(image.numpy_view())
            #print(image_copy)

            annotated_image, objCenter , imageWidth , imageLabel = self.visualize(image_copy, detection_result)
            rgb_annotated_image = cv2.cvtColor(annotated_image, cv2.COLOR_BGR2RGB)
            if imageLabel!= '':
                print(f"object centre: {objCenter}")
                print(f"image width: {imageWidth}")

                print(f"image name: {imageLabel}")
                with open(dump_file) as outfile:
                    detected = json.load(outfile)
                if detected["default"] == True:
                    if (detected["detected"] != True):
                        detected["detected"] = True

                    with open(dump_file, 'w') as outfile:
                        json.dump(detected ,outfile)

                rospy.loginfo("Object Found")
            else:
                 
                with open(dump_file) as outfile:
                    detected = json.load(outfile)
                        
                if (detected["detected"] == True):
                    detected["detected"] = False

                with open(dump_file, 'w') as outfile:
                    json.dump(detected ,outfile)

            #cv2.imshow("",image_copy)
            # print(image_copy[0])
            cv2.imshow("",rgb_annotated_image)
            cv2.waitKey(1)
            #rospy.spinOnce()
        elif self.count > 50:
            self.count = 0
        else:
            self.count += 1

    #iterate through 
    #407.0, 244.5
    # def getPostion(self,):
    #     # Assume we have a pixel coordinate (u,v) in an image
    #     u, v = 100, 200

    #     # Assume we have the camera's intrinsic and extrinsic parameters
    #     fx, fy = 500, 500  # focal length in x and y direction
    #     cx, cy = 320, 240  # image center coordinates
    #     R = np.eye(3)      # rotation matrix
    #     T = np.zeros((3,1)) # translation vector

    #     # Compute the direction and origin of the ray
    #     ray_dir = np.array([(u - cx) / fx, (v - cy) / fy, 1.0])
    #     ray_origin = np.zeros((3,))

    #     # Compute the 3D position of the point in the real world
    #     point_3d = np.dot(np.linalg.inv(R), ray_dir) * np.linalg.norm(T)

    #     # Convert the 3D position into ROS coordinates
    #     point_ros = np.array([point_3d[0], -point_3d[1], point_3d[2]])

    #     # Print the resulting position in ROS coordinates
    #     print(point_ros)



# from file/ sensor -> 





if __name__ == '__main__':
    rospy.init_node('tiago_detection')
    print("Starting object detector")
    global dump_file 
    dump_file= get_json_file()
    with open(dump_file) as outfile:
        detected = json.load(outfile)
            
    if (detected["detected"] == True):
        detected["detected"] = False

    with open(dump_file, 'w') as outfile:
        json.dump(detected ,outfile)




    tiagoDetection = tiagoDetection()
    
    rospy.spin()