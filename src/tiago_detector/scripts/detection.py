#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Float32 ,String, Header
from geometry_msgs.msg import Point, Quaternion, Pose, PoseStamped
import mediapipe as mp
import numpy as np
import cv2
from cv_bridge import CvBridge
import ros_numpy
import os
import pathlib
from mediapipe.tasks.python import vision
import json
import sensor_msgs.point_cloud2 as pc2


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
        self.point_cloud = None
        self.models = self.get_models()
        self.objCenter_pub = rospy.Publisher('image_centre', Float32, queue_size=10)
        self.imageWidth_pub = rospy.Publisher('image_width', Float32, queue_size=10)
        self.imageLabel_pub = rospy.Publisher('image_label', String, queue_size=10)
        rospy.Subscriber("/xtion/rgb/image_raw", Image, self.callback, queue_size=1, buff_size=2058)
        rospy.Subscriber("/xtion/depth_registered/points", PointCloud2, self.point_callback,queue_size=2, buff_size=2058)
        
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

    def point_callback(self, data):
        self.point_cloud = data

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
                self.publishCentre(objCenter)
            else:
                 
                with open(dump_file) as outfile:
                    detected = json.load(outfile)
                        
                if (detected["detected"] == True):
                    detected["detected"] = False

                with open(dump_file, 'w') as outfile:
                    json.dump(detected ,outfile)

            cv2.imshow("",rgb_annotated_image)
            cv2.waitKey(1)
            #rospy.spinOnce()
        elif self.count > 50:
            self.count = 0
        else:
            self.count += 1

    #iterate through 
    #407.0, 244.5

    def publishCentre(self,objCenter):
        print("publishing centre")
        x = int(objCenter[0])
        y = int(objCenter[1])
        # print(f"x : {x} y: {y}")
        gen = pc2.read_points(self.point_cloud, uvs=[(x,y)], skip_nans=True)
        print(gen)
        for p in gen:
           print(f"x : {p[0]} y: {p[1]} z : {p[2]}")
           q = quaternion_from_euler(1.5707, -1.5707, 1.5707)
           if isinstance(p[0], float) and isinstance(p[1], float) and isinstance(p[2], float):
                my_header = Header(stamp=self.point_cloud.header.stamp, frame_id=self.point_cloud.header.frame_id)
                my_pose = Pose(position=Point(x=p[0], y=p[1], z=p[2]), orientation=Quaternion(
                    x=q[0], y=q[1], z=q[2], w=q[3]))
                pose_stamped = PoseStamped(header=my_header, pose=my_pose)
                #self.pub.publish(pose_stamped)
           



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