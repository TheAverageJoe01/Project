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
        # If no JSON file is found, print a message to the console
        print("No JSON file found in current directory.")

class tiagoDetection:

    def __init__(self):
        # Initialize count, objCenter, imageWidth, bridge, imageLabel, and pointCloud variables
        self.count = 0
        self.objCenter = (0, 0)
        self.imageWidth = 0
        self.bridge = CvBridge()
        self.imageLabel = ''
        self.pointCloud = None
        
        # Get the models from get_models() method
        self.models = self.get_models()

        # Create publishers for 'objectCentre', 'image_centre', 'image_width', and 'image_label'
        self.pub = rospy.Publisher("objectCentre", PoseStamped, queue_size=10)
        self.objCenter_pub = rospy.Publisher('image_centre', Float32, queue_size=10)
        self.imageWidth_pub = rospy.Publisher('image_width', Float32, queue_size=10)
        self.imageLabel_pub = rospy.Publisher('image_label', String, queue_size=10)
        
        # Subscribe to ROS topics "/xtion/rgb/image_raw" and "/xtion/depth_registered/points"
        # and call 'callback' and 'point_callback' methods respectively
        rospy.Subscriber("/xtion/rgb/image_raw", Image, self.callback, queue_size=1, buff_size=2058)
        rospy.Subscriber("/xtion/depth_registered/points", PointCloud2, self.point_callback, queue_size=2, buff_size=2058)
        
    def get_models(self):
        # Set the path to the directory containing the models
        MODELDIR = pathlib.Path(os.path.join(pathlib.Path(__file__).parent.absolute(),'models')).glob('**/*')

        # Create an empty list to store the models
        models = list()

        # Loop through all files in the directory and append the file paths to the models list
        for i in MODELDIR:
            # Check if the file is a regular file
            if i.is_file():
                # Append the file path to the models list
                models.append(i)

        # Return the list of model file paths
        return models
    
    
    
    def visualize(self,image, detection_result) -> np.ndarray:
        #set some constants for visualisation
        MARGIN = 10
        ROW_SIZE = 10
        FONT_SIZE = 1
        FONT_THICKNESS = 1
        TEXT_COLOR = (255, 0, 0)
        #initialise variables
        centre = (0, 0)
        imageWidth = 0
        imageLabel = ''
        #define a list of unwanted labels
        unwanted = ["bed","tv","laptop","train","book"]
        #loop through the detected objects
        for detection in detection_result.detections:
            category = detection.categories[0]
            category_name = category.category_name
            #a check to see if the detected object is in the list of unwanted labels
            if category_name in unwanted:
                break
            # draw the bounding box around the detected object
            bbox = detection.bounding_box
            start_point = bbox.origin_x, bbox.origin_y
            end_point = bbox.origin_x + bbox.width, bbox.origin_y + bbox.height
            centre = bbox.origin_x + (bbox.width * 0.5), bbox.origin_y + (bbox.height * 0.5)
            imageWidth = bbox.width
            cv2.rectangle(image, start_point, end_point, TEXT_COLOR, 3)

            
            imageLabel = category_name
            # Publish the object centre, image width, and image label to the corresponding topics
            self.objCenter_pub.publish(centre[0])
            self.imageWidth_pub.publish(imageWidth)
            self.imageLabel_pub.publish(imageLabel)

            #draw the label name and probability
            probability = round(category.score, 2)
            result_text = category_name + ' (' + str(probability) + ')'
            text_location = (MARGIN + bbox.origin_x,
                            MARGIN + ROW_SIZE + bbox.origin_y)
            cv2.putText(image, result_text, text_location, cv2.FONT_HERSHEY_PLAIN,
                        FONT_SIZE, TEXT_COLOR, FONT_THICKNESS)

        return image , centre , imageWidth , imageLabel

    def point_callback(self, data):
        # Store the point cloud data
        self.pointCloud = data

    def callback(self, data):
        if self.count == 0:
                

            # convert the ROS Image to a numpy array
            imageNP = ros_numpy.numpify(data)

            #import the base options from mediapipe
            BaseOptions = mp.tasks.BaseOptions
            #import the object detector options from mediapipe
            ObjectDetectorOptions = mp.tasks.vision.ObjectDetectorOptions
            #import the running mode from mediapipe
            VisionRunningMode = mp.tasks.vision.RunningMode

            efficientdet_lite0= self.models[0]
            #print(f"Model - {efficientdet_lite0}")

            #create and instance of the ObjectDetectorOptions class, using the first model in the models list
            options = ObjectDetectorOptions( 
                base_options=BaseOptions(model_asset_path= efficientdet_lite0),
                max_results=5,
                score_threshold=0.285,
                running_mode=VisionRunningMode.IMAGE)

            # create an instance of the ObjectDetector class using the options
            detector = vision.ObjectDetector.create_from_options(options)


            # create an instance of the mp.Image class using the image numpy array
            image = mp.Image(image_format=mp.ImageFormat.SRGB, data=imageNP)

            # detect the objects in the image using the detector
            detection_result = detector.detect(image)

            # Process the detection result. In this case, visualize it.
            image_copy = np.copy(image.numpy_view())
            #print(image_copy)

            # call the visualize function with the image copy and the detection result to obtain the annotated image and the object center, image width, and image label
            annotated_image, objCenter , imageWidth , imageLabel = self.visualize(image_copy, detection_result)
            # convert the annotated image to RGB color space
            rgb_annotated_image = cv2.cvtColor(annotated_image, cv2.COLOR_BGR2RGB)
            #if an object label was detected
            if imageLabel!= '':
                print(f"object centre: {objCenter}")
                print(f"image width: {imageWidth}")

                print(f"image name: {imageLabel}")
                #open dump file
                with open(dump_file) as outfile:
                    # load the JSON data from the dump file into the detected dictionary
                    detected = json.load(outfile)
                if detected["default"] == True:
                    if (detected["detected"] != True):
                        detected["detected"] = True
                     # open the dump file for writing
                    with open(dump_file, 'w') as outfile:
                        # write the updated detected dictionary to the dump file
                        json.dump(detected ,outfile)
                # log the message "Object Found" using rospy.loginfo()
                rospy.loginfo("Object Found")
                # publish the object center 
                self.publishCentre(objCenter)
            # if no object label was detected
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
        # Get the x and y coordinates of the object center
        x = int(objCenter[0])
        y = int(objCenter[1])
        # print(f"x : {x} y: {y}")
        # Read the point cloud at the (x,y) position and skip NaNs
        gen = pc2.read_points(self.pointCloud, uvs=[(x,y)], skip_nans=True)
        print(gen)
        # Loop over the generator and print the x, y, and z coordinates of each point
        for p in gen:
           
            print(f"x : {p[0]} y: {p[1]} z : {p[2]}")
            # Define a quaternion that rotates 90 degrees around the x axis, -90 degrees around the y axis, and 90 degrees around the z axis
            q = quaternion_from_euler(1.5707, -1.5707, 1.5707)
            if isinstance(p[0], float) and isinstance(p[1], float) and isinstance(p[2], float):
                # Create a header object with the timestamp and frame ID of the point cloud
                centreHeader = Header(stamp=self.pointCloud.header.stamp, frame_id=self.pointCloud.header.frame_id)
                 # Create a pose object with the position and orientation of the detected object
                centrePose = Pose(position=Point(x=p[0], y=p[1], z=p[2]), orientation=Quaternion(
                    x=q[0], y=q[1], z=q[2], w=q[3]))
                 # Create a PoseStamped message with the header and pose
                pose_stamped = PoseStamped(header=centreHeader, pose=centrePose)
                # Publish the pose_stamped message to the topic
                self.pub.publish(pose_stamped)
                    #self.publishCentre(objCenter)
           



# from file/ sensor -> 





if __name__ == '__main__':

    rospy.init_node('tiago_detection')
    print("Starting object detector")
    # Retrieve the path of the JSON dump file and store it in a global variable named 'dump_file'
    global dump_file 
    dump_file= get_json_file()
    # Open the JSON dump file in read mode and load its contents into a dictionary named 'detected'
    with open(dump_file) as outfile:
        detected = json.load(outfile)
    
    # If the 'detected' dictionary contains a key named 'detected' with a value of True, set it to False
    if (detected["detected"] == True):
        detected["detected"] = False

    # Overwrite the contents of the JSON dump file with the updated 'detected' dictionary
    with open(dump_file, 'w') as outfile:
        json.dump(detected ,outfile)



    # Instantiate an object of the 'tiagoDetection' class and store it in a variable named 'tiagoDetection'
    tiagoDetection = tiagoDetection()
    
    # Enter a loop and continue running until the node is shutdown
    rospy.spin()