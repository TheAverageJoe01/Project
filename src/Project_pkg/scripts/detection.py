#!/usr/bin/env python

import rospy
import mediapipe as mp
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from mediapipe.tasks import python
from mediapipe.tasks.python import vision

MARGIN = 10  # pixels
ROW_SIZE = 10  # pixels
FONT_SIZE = 1
FONT_THICKNESS = 1
TEXT_COLOR = (255, 0, 0)  # red

def visualize(image, detection_result) -> np.ndarray:
    """Draws bounding boxes on the input image and return it.
    Args:
        image: The input RGB image.
        detection_result: The list of all "Detection" entities to be visualize.
    Returns:
        Image with bounding boxes.
    """
    centre = 0
    imageWidth = 0
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
        probability = round(category.score, 2)
        result_text = category_name + ' (' + str(probability) + ')'
        text_location = (MARGIN + bbox.origin_x,
                         MARGIN + ROW_SIZE + bbox.origin_y)
        cv2.putText(image, result_text, text_location, cv2.FONT_HERSHEY_PLAIN,
                    FONT_SIZE, TEXT_COLOR, FONT_THICKNESS)

    return image , centre , imageWidth


def object_detector_node():
    rospy.init_node('object_detector_node', anonymous=True)
    bridge = CvBridge()

    # Load the input image.
    cup = '/home/joe/Project/src/Project_pkg/include/images/cup.jpg'
    image = cv2.imread(cup)
    # Convert image to RGB
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    image_msg = bridge.cv2_to_imgmsg(image, "rgb8")

    # Create object detector
    efficientdet_lite0= '/home/joe/Project/src/Project_pkg/include/models/efficientdet_lite0_uint8.tflite'
    options = vision.ObjectDetectorOptions(
        base_options=mp.tasks.BaseOptions(model_asset_path=efficientdet_lite0),
        max_results=5,
        score_threshold=0.5,
        running_mode=mp.tasks.vision.RunningMode.IMAGE)
    detector = vision.ObjectDetector.create_from_options(options)

    # Detect objects in the input image.
    detection_result = detector.detect(mp.Image(image_msg))

    # Process the detection result. In this case, visualize it.
    image_copy = np.copy(image)
    annotated_image, imageCentre , imageWidth = visualize(image_copy, detection_result)
    rgb_annotated_image = cv2.cvtColor(annotated_image, cv2.COLOR_BGR2RGB)
    print(imageCentre)
    print(imageWidth)
    cv2.imshow("", rgb_annotated_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == '__main__':
    try:
        object_detector_node()
    except rospy.ROSInterruptException:
        pass
