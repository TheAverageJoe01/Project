"""
test code to draw bounding boxes on the input image and return it and other values



TO DO list:
- create a pub and sub for relative information 
- convert output info into usual data formatting pixels into cm 
- 




"""
import rospy
from sensor_msgs.msg import Image, CameraInfo
import mediapipe as mp
import numpy as np
import cv2
from mediapipe.tasks import python
from mediapipe.tasks.python import vision

from cv_bridge import CvBridge, CvBridgeError
import os
import pathlib


class tiagoDetection:
    def __init__(self):
        # 
        self.imageCentre = 0
        self.imageWidth = 0
        self.bridge = CvBridge()
        self.imageLabel = 'a'
        self.models = self.get_models()
        #self.annotated_image, self.imageCentre , self.imageWidth , self.imageLabel = self.visualize(image_copy, detection_result)
        self.loopRate = rospy.Rate(10)



        rospy.Subscriber("/ /rgb/image_rect_color", Image, self.callback, queue_size=1, buff_size=2058)




    def get_models(self):
        MODELDIR = pathlib.Path(os.path.join(pathlib.Path(__file__).parent.absolute(),'models')).glob('**/*')
        models = list()
        for i in MODELDIR:
            if i.is_file():
                models.append(i)
        return models



    def visualize(self,image, detection_result) -> np.ndarray:
        """Draws bounding boxes on the input image and return it.
        Args:
            image: The input RGB image.
            detection_result: The list of all "Detection" entities to be visualize.
        Returns:
            Image with bounding boxes.
        """

        MARGIN = 10  # pixels
        ROW_SIZE = 10  # pixels
        FONT_SIZE = 1
        FONT_THICKNESS = 1
        TEXT_COLOR = (255, 0, 0)  # red
        centre = self.imageCentre
        imageWidth = self.imageWidth
        imageLabel = self.imageLabel
        for detection in detection_result.detections:
            # Draw bounding_box
            bbox = detection.bounding_box
            start_point = bbox.origin_x, bbox.origin_y
            end_point = bbox.origin_x + bbox.width, bbox.origin_y + bbox.height
            # getting the centre of the bounding box
            centre = bbox.origin_x + (bbox.width * 0.5), bbox.origin_y + (bbox.height * 0.5)
            imageWidth = bbox.width
            cv2.rectangle(image, start_point, end_point, TEXT_COLOR, 3)

            # Draw label and score
            category = detection.categories[0]
            category_name = category.category_name
            imageLabel = category_name
            probability = round(category.score, 2)
            result_text = category_name + ' (' + str(probability) + ')'
            text_location = (MARGIN + bbox.origin_x,
                            MARGIN + ROW_SIZE + bbox.origin_y)
            cv2.putText(image, result_text, text_location, cv2.FONT_HERSHEY_PLAIN,
                        FONT_SIZE, TEXT_COLOR, FONT_THICKNESS)

        return image , centre , imageWidth , imageLabel

    def callback(self, data):
        cv2.namedWindow("tiagoImage")
        imageCV = self.bridge.imgmsg_to_cv2(data, "bgr8")
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
        image = mp.Image.create_from_file(imageCV)
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
        cv2.imshow("",rgb_annotated_image)
        cv2.waitKey(0)
    
    def cv2tomp(self, image):
        return 0 




# from file/ sensor -> 


IMAGEDIR = pathlib.Path(os.path.join(pathlib.Path(__file__).parent.absolute(),'images')).glob('**/*')



images = {}
for i in IMAGEDIR:
  if i.is_file():
    images[i.stem] = str(i)



image = mp.Image.create_from_file(images["potted_plant"])


print(images)




if __name__ == '__main__':
    print("Starting object detector")
    tiagoDetection = tiagoDetection()
    
    print(image)
    #tiagoDetection.callback(image)
