#!/usr/bin/env python3


#imports
import rospy
from geometry_msgs.msg import PoseStamped, Pose, Twist, Point
import math 
from play_motion_msgs import PlayMotionAction, PlayMotionGoal
from actionlib import SimpleActionClient
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from Project_pkg import Absolute, absoluteResponse
from Project_pkg import objectApproach, objectApproachResponse
from std_srvs.srv import Empty, EmptyRequest 


#global variables
global actionClient
global pubHeadcontroller
global sub_target_rel_pose
global head_2_movement
global objectFound
global playMotionG
global count
global displacement
global object_rel_pose
global object_abs_pose

def defaultPose():
    # go to the default starting position of the tiago robot

    global actionClient 
    global playMotionG
    #create the status for default pose
    rospy.loginfo("Going to default pose")

    #define play motion config
    playMotionG = PlayMotionGoal()
    playMotionG.motion_name = "default_pose"
    playMotionG.skip_planning = False
    #send the goal
    actionClient.send_goal(playMotionG)
    #wait for the result
    actionClient.wait_for_result()
    #get the result
    result = actionClient.get_result()
    rospy.loginfo(result)


def headMovement():

    #global variables
    global pubHeadcontroller
    global head_2_movement
    global objectFound

    #create the status for head movement
    rospy.loginfo("moving head")

    #creating a while loop until the object is found
    while objectFound == False:
        #trajectory_msgs
        traj = JointTrajectory()
        # define the trajectory points
        traj.joint_names = ["head_joint_1", "head_joint_2"]

        trajPoints = JointTrajectoryPoint()
        #change trajectory coordinates on the z axis
        trajPoints.positions = [0.0, "head_joint_2"]
        trajPoints.time_from_start = rospy.Duration(1.0)
        traj.points.append(trajPoints)

        #send the trajectory
        pubHeadcontroller.publish(traj)
        #wait for the result
        rospy.sleep(1.0)
        # define the head movement 
        head_2_movement = max(-1, head_2_movement - 0.1)


def objectPositionRel():

    #global variables
    global objectFound
    global sub_target_rel_pose
    global object_rel_pose
    global count

    object_rel_pose = msg

    #call absolute position function 
    objectPositionAbs()

    count+=1
    if count == 25:
        objectFound == True


def robotPrep():

    #global variables
    global actionClient 
    global playMotionG



    #create the status 
    rospy.loginfo("preparing robot")

    #define play motion config
    playMotionG = PlayMotionGoal()
    playMotionG.motion_name - 'preparation'
    playMotionG.skip_planning = False
    #send the goal
    actionClient.send_goal(playMotionG)
    #wait for the result
    actionClient.wait_for_result()
    #get the result
    result = actionClient.get_result()
    rospy.loginfo(result)

    rospy.loginfo("preparation done")


def adjustHead():
    #global variables
    global displacement

    #create the status
    rospy.loginfo("found displacement: " + str(displacement))

    pub_vel = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=1)

    velocity = Twist()

    #defining the angular velocity
    #moving side to side 
    for i in range(21):
        velocity.angular.z = -math.pi/4
        pub_vel.publish(velocity)
        rospy.sleep(0.1)
    
    velocity.angular.z = 0
    pub_vel.publish(velocity)
    rospy.sleep(0.1)


    #defining the linear velocity to move the robot to the correct position
    for i in range(displacement *10/0.25):
        velocity.linear.x = 0.25
        pub_vel.publish(velocity)
        rospy.sleep(0.1)
        
    velocity.linear.x = 0
    pub_vel.publish(velocity)
    rospy.sleep(0.1)

    #defining the angular velocity
    #moving to the table 
    for i in range(20):
        velocity.angular.z = -math.pi/4
        pub_vel.publish(velocity)
        rospy.sleep(0.1)

    velocity.angular.z = 0
    pub_vel.publish(velocity)
    rospy.sleep(0.1)

    rospy.loginfo("adjusting position done")

def objectPositionAbs():
    #global variables
    global object_abs_pose
    global object_rel_pose

    try:
        rel_to_abs = rospy.ServiceProxy('/Absolute', absoluteResponse)
        msg = PoseStamped(pose = object_rel_pose )
        msg.header.frame_id = 'xtion_rgb_frame'

        object_abs_pose = rel_to_abs(msg).absolute_pose.pose
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == '__main__':

    #node initialization
    rospy.init_node('pickClient')

    actionClient = SimpleActionClient('play_motion', PlayMotionAction)

    #time out in case of lack of connection
    if not actionClient.wait_for_server(rospy.Duration(5.0)):
        rospy.logerr("Action server not available")
        exit(1)

    playMotionG = PlayMotionGoal()

    # call the default pose function
    defaultPose()

    #publishing the head controller 
    pubHeadcontroller = rospy.Publisher('/head_controller/command', JointTrajectory, queue_size=1)

    #subscribing to the target position
    sub_target_rel_pose = rospy.subscriber('/target_pose/relative', Pose, objectPositionRel)

    #create an empty value 
    head_2_movement = 0
    objectFound = False
    count = 0

    #call the head movement function
    headMovement()

    rospy.wait_for_service('/absolute')


    #define displacement 
    displacement = 0.1 - object_abs_pose.position.y

    if displacement > 0:
        adjustHead()
    
    robotPrep()
    rospy.sleep(2)

    rospy.wait_for_service('/object_approach')
    try:
        sub_target_rel_pose.unregister()
        objectApproach = rospy.ServiceProxy('/object_approach', objectApproach)
        objectApproach(object_abs_pose)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        exit(1)
    
    rospy.wait_for_service('/pickUpObject', Empty)
    try:
        sub_target_rel_pose.unregister()
        pickUpObject = rospy.ServiceProxy('/pickUpObject', Empty)
        pickUpObject()
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        exit(1)
        