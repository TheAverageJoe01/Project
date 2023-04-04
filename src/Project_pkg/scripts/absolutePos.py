#!/usr/bin/env python3


#imports 
import rospy
import tf2_ros
from tf.transformations import *
from tf2_geometry_msgs import *
import numpy_ros
from geometry_msgs.msg import Pose, Quaternion, PointStamped
from Project_pkg import Absolute, AbsoluteResponse 


#global variables
global bufferTF 
global listener

def absolutePose(posRelative):

    #generate log
    rospy.loginfo("Requesting absolute pose")

    #relative position transformation between the xtion_rgb_frame and the base footprint
    baseRel = bufferTF.lookup_transform('base_footprint', posRelative.relative_pose.header.frame_id, rospy.Time())

    pointRelative = PointStamped(point = posRelative.relative_pose.pose.position)


    #absolute model
    posAbsolute = Pose()

    posAbsolute.position = do_transform_point(pointRelative, baseRel).point

    #calculating rotation between the xtion_rgb_frame and the base footprint
    quat0 = [baseRel.transform.rotation.x, baseRel.transform.rotation.y, 
             baseRel.transform.rotation.z, baseRel.transform.rotation.w]
    
    #calculating orientation to get the absolute pose
    quat1 = [posRelative.relative_pose.pose.orientation.x, posRelative.relative_pose.pose.orientation.y, 
             posRelative.relative_pose.pose.orientation.z, posRelative.relative_pose.pose.orientation.w]
    
    #multiplying the two quaternions
    quat = qauternion_multiply(quat0, quat1)

    #getting the quaternion coordinates
    posAbsolute.orientation = Quaternion(quat[0], quat[1], quat[2], quat[3])

    #return the relative and absolute pose in returnPose
    returnPose = AbsoluteResponse()
    #use a reference frame for base_footprint
    returnPose.absolute_pose.header.frame_id = 'base_footprint'
    print("returnPose")
    return returnPose

if __name__ == '__main__':

    #node initialization
    rospy.init_node('absolutePose', anonymous=True)

    absouluteService = rospy.Service('absolutePose', Absolute, absolutePose)

    #track multiple coordinate frames over time
    bufferTF = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(bufferTF)

    rospy.loginfo("AbsolutePose node started")
    rospy.spin()