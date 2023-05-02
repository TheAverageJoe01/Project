#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32 ,String
import mediapipe as mp
import numpy as np
import cv2
from cv_bridge import CvBridge
import os
import pathlib
from mediapipe.tasks.python import vision

class tiagoDetection:
    def __init__(self):
        self.count = 0
        self.imageCentre = (0, 0)
        self.imageWidth = 0
        self.bridge = CvBridge()
        self.imageLabel = ''
        self.models = self.get_models()
        self.imageCentre_pub = rospy.Publisher('image_centre', Float32, queue_size=10)
        self.imageWidth_pub = rospy.Publisher('image_width', Float32, queue_size=10)
        self.imageLabel_pub = rospy.Publisher('image_label', String, queue_size=10)
        rospy.Subscriber("/xtion/rgb/image_rect_color", Image, self.callback, queue_size=1, buff_size=2058)
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
        for detection in detection_result.detections:
            bbox = detection.bounding_box
            start_point = bbox.origin_x, bbox.origin_y
            end_point = bbox.origin_x + bbox.width, bbox.origin_y + bbox.height
            centre = bbox.origin_x + (bbox.width * 0.5), bbox.origin_y + (bbox.height * 0.5)
            imageWidth = bbox.width
            cv2.rectangle(image, start_point, end_point, TEXT_COLOR, 3)

            category = detection.categories[0]
            category_name = category.category_name
            imageLabel = category_name

            self.imageCentre_pub.publish(centre[0])
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
                
            imageCV = self.bridge.imgmsg_to_cv2(data, "bgr8")
            #create a counter 
            imageNP = np.asarray(imageCV)
            print(imageNP)
            BaseOptions = mp.tasks.BaseOptions
            ObjectDetectorOptions = mp.tasks.vision.ObjectDetectorOptions
            VisionRunningMode = mp.tasks.vision.RunningMode

            efficientdet_lite0= self.models[0]
            print(f"Model - {efficientdet_lite0}")

            options = ObjectDetectorOptions( 
                base_options=BaseOptions(model_asset_path= efficientdet_lite0),
                max_results=5,
                score_threshold=0.5,
                running_mode=VisionRunningMode.IMAGE)

            detector = vision.ObjectDetector.create_from_options(options)


            # Load the input image.
            image = mp.Image(image_format=mp.ImageFormat.SRGB, data=imageNP)

            #  Detect objects in the input image.
            detection_result = detector.detect(image)

            # Process the detection result. In this case, visualize it.
            image_copy = np.copy(image.numpy_view())
            #print(image_copy)

            annotated_image, imageCentre , imageWidth , imageLabel = self.visualize(image_copy, detection_result)
            rgb_annotated_image = cv2.cvtColor(annotated_image, cv2.COLOR_BGR2RGB)
            print(f"image centre: {imageCentre}")
            print(f"image width: {imageWidth}")
            print(f"image name: {imageLabel}")
            #cv2.imshow("",rgb_annotated_image)
            cv2.imshow("",imageCV)
            cv2.waitKey(1)
            #rospy.spinOnce()
        elif self.count > 50:
            self.count = 0
        else:
            self.count += 1

    




# from file/ sensor -> 





if __name__ == '__main__':
    rospy.init_node('tiago_detection')
    print("Starting object detector")
    tiagoDetection = tiagoDetection()
    #tiagoDetection.callback(image)
    rospy.spin()