#! /usr/bin/env python3

#imports
from operator import sub
import rospy 
from tf.transformations import *
from tf2_geometry_msgs import Pose, Quaternion, Point
from scipy.spatial.transform import Rotation as R
import moveit_commander
import sys
from std_srvs.srv import Empty, EmptyResponse
from Project_pkg import objectApproach, objectApproachResponse
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


#global variables
global robot
global sub_target_abs_pose
global scene 
global move_group 
global grasp_pose 

def moveToobject(objectPose):


    #global variables
    global moveGroup
    global graspPose

    #define quaternion coordinates of the object found on the table
    objectPose.pose.orientation = Quaternion(0.5, 0.5, 0.5, 0.5)

    #save the grasp pose
    graspPose = copy.deepcopy(objectPose.pose)
    #define the distance between the object and the table
    graspPose.position.y = 0.22

    #preGraspPose 
    objectPose.pose.position.y = -0.2
    objectPose.pose.position.z += 0.1


    #create robot status 
    rospy.loginfo("Attempting to reach the object")
    rospy.loginfo([objectPose.pose.position.x, objectPose.pose.position.y, objectPose.pose.position.z])

    moveGroup.set_pose_target(objectPose.pose)
    moveGroup.go(wait=True)

    moveGroup.stop()

    moveGroup.clear_pose_targets()

    response = objectApproachResponse()
    response.success = True
    return response


def pickObject(msg):

    #global variables
    global moveGroup
    global graspPose


    #move to grasp
    rospy.loginfo("move to grasp position")
    moveGroup.set_pose_target(graspPose)
    moveGroup.go(wait=True)
    moveGroup.stop()
    moveGroup.clear_pose_targets()

    rospy.loginfo("closing gripper")
    closeGripper()
    rospy.sleep(1)

    #define post grip pose
    postGripPose = copy.deepcopy(graspPose)
    postGripPose.position.z +=0.3

    #define post grip pose
    rospy.loginfo("post grip position")
    moveGroup.set_pose_target(postGripPose)
    moveGroup.go(wait=True)
    moveGroup.stop()
    moveGroup.clear_pose_targets()

    #move to grasp position
    rospy.loginfo("move to grasp position")
    moveGroup.set_pose_target(graspPose)
    moveGroup.go(wait=True)
    moveGroup.stop()
    moveGroup.clear_pose_targets()

    rospy.loginfo("closing gripper")
    closeGripper()
    rospy.sleep(1)

    #define post grip pose
    postGripPose = copy.deepcopy(graspPose)
    postGripPose.position.z +=0.3

    #define post grip pose
    rospy.loginfo("post grip position")
    moveGroup.set_pose_target(postGripPose)
    moveGroup.go(wait=True)
    moveGroup.stop()
    moveGroup.clear_pose_targets()

    #move to grasp position
    rospy.loginfo("move to grasp position")
    moveGroup.set_pose_target(graspPose)
    moveGroup.go(wait=True)
    moveGroup.stop()
    moveGroup.clear_pose_targets()

    rospy.loginfo("open gripper")
    openGripper()
    rospy.sleep(1)

    rospy.loginfo("finished picking object")

def closeGripper():

    # publisher to close gripper
    pubGrippercontroller = rospy.Publisher('/gripper_controller/command', JointTrajectory, queue_size=1)

    #loop until gripper is closed
    for i in range(10):
        traj = JointTrajectory()
        #call joint group for object 
        traj.joint_names = ['gripper_left_finger_joint', 'gripper_right_finger_joint']

        trajPoints = JointTrajectoryPoint()
        #define the distance between the two grippers 
        trajPoints.positions = [0.0, 0.0]
        trajPoints.time_from_start = rospy.Duration(1.0)
        traj.points.append(trajPoints)
        pubGrippercontroller.publish(traj)
        rospy.sleep(0.1)


def openGripper():
    # publisher to close gripper
    pubGrippercontroller = rospy.Publisher('/gripper_controller/command', JointTrajectory, queue_size=1)

    #loop until gripper is closed
    for i in range(10):
        traj = JointTrajectory()
        #call joint group for object 
        traj.joint_names = ['gripper_left_finger_joint', 'gripper_right_finger_joint']

        trajPoints = JointTrajectoryPoint()
        #define the distance between the two grippers 
        trajPoints.positions = [0.044, 0.044]
        trajPoints.time_from_start = rospy.Duration(1.0)
        traj.points.append(trajPoints)
        pubGrippercontroller.publish(traj)
        rospy.sleep(0.1)


if __name__ == '__main__':

    #initialise ros node
    rospy.init_node('pickObject')
    
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    move_group = moveit_commander.MoveGroupCommander("arm_torso")

    approachObjectservice = rospy.Service('/objectApproach', objectApproach, moveToobject)

    graspObjectservice = rospy.Service('/pickObject', Empty, pickObject)

    rospy.loginfo("service is ready")

    openGripper()

    rospy.spin()