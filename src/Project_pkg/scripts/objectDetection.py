#!/usr/bin/env python3
"""
description:
    used to find objects in the scene

subscribes:
    /xtion/rgb/image_raw
publishes:
    /target_pose/relative
    /target_pose/relative/stamped
"""

#imports
from sensor_msgs.msg import Image
import rospy
import ros_numpy
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
import mediapipe as mp
from scipy.spatial.transform import Rotation as R

mpObjectron = mp.solutions.objectron

def detection(image):

    # define the publishers as variables inside the function
    pub_target_rel_pose = rospy.Publisher("/target_rel_pose", Pose, queue_size=1)
    pub_target_rel_pose_stamped = rospy.Publisher("/target_rel_pose_stamped", PoseStamped, queue_size=1)

    # define the static model carried from mediaPipe objectron
    with mpObjectron.Objectron(
            static_image_mode=False,
            maxObjects=5,  # set maxObjects to the maximum number of objects to detect
            minDetectionConfidence=0.5,
            modelN='cup') as objectron:

        # get the output as an image
        outcome = objectron.process(ros_numpy.numpify(image))

        for i, detected_object in enumerate(outcome.detected_objects):

            # create a message to publish
            rospy.loginfo(f"objectron detected, object {i}")

            messagePose = Pose()

            # position and orientation
            pos = detected_object.translation
            print(-pos[2], pos[0], pos[1])

            # rotation and quaternion
            quat = R.from_matrix(detected_object.rotation_matrix).as_quat()

            # finding orientation of the object with quaternions
            messagePose.orientation.x = quat[0]
            messagePose.orientation.y = quat[1]
            messagePose.orientation.z = quat[2]
            messagePose.orientation.w = quat[3]
            # finding position of the object with transformation
            messagePose.position.x = -pos[2]
            messagePose.position.y = pos[0]
            messagePose.position.z = pos[1]

            # publishing the message
            pub_target_rel_pose.publish(messagePose)

            poseStamped = PoseStamped(Pose=messagePose)
            # poseStamped used as a frame id i.e frame of the camera
            poseStamped.header.frame_id = "camera_link"

            # publishing the message
            pub_target_rel_pose_stamped.publish(poseStamped)


if __name__ == '__main__':
    rospy.init_node('objectron_detection', anonymous=True)

    #subscribe to camera for info
    cameraSub = rospy.Subscriber("/camera/rgb/image_raw", Image, detection)

    rospy.spin()