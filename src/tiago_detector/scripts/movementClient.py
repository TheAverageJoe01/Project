#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose, Twist, PoseStamped, Point
import math
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from actionlib import SimpleActionClient
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import os

import json
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
    
    with open(dump_file) as outfile:
        detected = json.load(outfile)

    detected['default'] = True
    with open(dump_file, 'w') as outfile:
        json.dump(detected, outfile)


    rospy.loginfo("Done.")


def move_head():
    # Define global variabile
    global pub_head_controller
    global head_2_movement
    global object_found

    # generate status
    rospy.loginfo("Moving head")
    

    object_found = False
    # loop function until object was found
    while object_found != True:
        with open(get_json_file()) as output_file:
            detected = json.load(output_file)
            object_found = detected['detected']
            if object_found == True:
                rospy.loginfo("Object found")
                rospy.sleep(0.2)
                break
                

        
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
        rospy.sleep(0.7)
        # Define head movement in a lowering cycle with -0.1 step to a max -1, until object is detected
        head_2_movement = max(-1, head_2_movement-0.1)

    rospy.loginfo("Done.")

def move_object_loop():
    object_found = False
    while object_found!= True:
        print(object_found)
        with open(dump_file) as outfile:
            detected = json.load(outfile)
            object_found = detected['detected']
        move_head()



if __name__ == '__main__':
   
    global dump_file
    dump_file = get_json_file()


    
    with open(dump_file) as outfile:
        detected = json.load(outfile)

    detected['default'] = False
    with open(dump_file, 'w') as outfile:
        json.dump(detected, outfile)
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
    
    
    
    head_2_movement = 0 
    
    move_head()