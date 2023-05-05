#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose, Twist, PoseStamped, Point
import math
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from actionlib import SimpleActionClient
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint



global action_client
global pub_head_controller
global sub_target_rel_pose
global head_2_movement
global object_found
global pmgoal
global count
global displacement
global object_rel_pose
global object_abs_pose


def home_position():
    global action_client
    global pmgoal
    # generate status
    rospy.loginfo("Go into the home position")

    # define Play motion configuration
    pmgoal = PlayMotionGoal()
    pmgoal.motion_name = 'home'
    pmgoal.skip_planning = False
    # send play motion goal --> home configuration & wait till reach position 
    action_client.send_goal_and_wait(pmgoal)

    rospy.loginfo("Done.")


def move_head():
    """
        move_head function use a while loop to move head joints until object is found by TIAGo
    """
    # Define global variabile
    global pub_head_controller
    global head_2_movement
    global object_found

    # generate status
    rospy.loginfo("Moving head")

    # loop function until object was found
    for i in range(100):
        # trajectory_msgs --> JointTrajectory
        trajectory = JointTrajectory()
        # Define joint in use in order to move head 
        trajectory.joint_names = ['head_1_joint', 'head_2_joint']

        # ***The action can be used to give a trajectory to the head expressed in several waypoints***. 
        trajectory_points = JointTrajectoryPoint()
        # change coordinate just along z axis
        trajectory_points.positions = [0.0, head_2_movement]
        # Define time action 
        trajectory_points.time_from_start = rospy.Duration(1.0)

        trajectory.points.append(trajectory_points)
        
        pub_head_controller.publish(trajectory)
        # interval to start next movement
        rospy.sleep(0.8)
        # Define head movement in a lowering cycle with -0.1 step to a max -1, until object is detected
        head_2_movement = max(-1, head_2_movement-0.1)

    rospy.loginfo("Done.")






if __name__ == '__main__':
   rospy.init_node('MovementClient')
   action_client = SimpleActionClient('/play_motion', PlayMotionAction)
   # time out in case of lack communication between client & server 
   if not action_client.wait_for_server(rospy.Duration(20)):
      rospy.logerr("Could not connect to /play_motion")
      exit()
      
      


   pmgoal = PlayMotionGoal()
   home_position()
   
    # publsih to head controller the join trajectory 
    
   pub_head_controller = rospy.Publisher('/head_controller/command', JointTrajectory, queue_size=1)
   

   # subscribe to target relative pose the position 
   #sub_target_rel_pose = rospy.Subscriber('/sofar/target_pose/relative', Pose, get_object_relative_pose)
   
   
   head_2_movement = 0 
   move_head()