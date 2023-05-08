#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose, Twist, PoseStamped, Point
import math
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from actionlib import SimpleActionClient
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import os
import json


# setting global variables
global actionClient
global PMG
global pubHeadcontroller
global headMovement
global objectFound


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



def homePosition():
    # declare global variables
    global actionClient
    global PMG
    # print a message indicating that the robot is going into the home position
    rospy.loginfo("Go into the home position")

    # Create a PlayMotionGoal object
    PMG = PlayMotionGoal()
    PMG.motion_name = 'home'
    PMG.skip_planning = False

    # Send the goal to the action server and wait for it to complete
    actionClient.send_goal_and_wait(PMG)
    
    # Update the JSON file to indicate that the default object is selected
    with open(dump_file) as outfile:
        detected = json.load(outfile)

    detected['default'] = True
    with open(dump_file, 'w') as outfile:
        json.dump(detected, outfile)

    # print a message indicating that the robot has finished moving to the home position
    rospy.loginfo("finished.")


def movingHead():
    # Define global variabile
    global pubHeadcontroller
    global headMovement
    global objectFound

    rospy.loginfo("Moving head")
    
    # Initialise the objectFound flag
    objectFound = False
    while objectFound != True:
        with open(get_json_file()) as output_file:
            detected = json.load(output_file)
            objectFound = detected['detected']
            # If an object is found, log the message and sleep for 0.2 seconds
            if objectFound == True:
                rospy.loginfo("Object found")
                rospy.sleep(0.2)
                
                break
            
        # Define the trajectory for moving the head
        trajectory = JointTrajectory()
        trajectory.joint_names = ['head_1_joint', 'head_2_joint']

        # Define the trajectory points
        trajectoryPoints = JointTrajectoryPoint()
        trajectoryPoints.positions = [0.0, headMovement]
        trajectoryPoints.time_from_start = rospy.Duration(1.0)
        trajectory.points.append(trajectoryPoints)
        
        # Publish the trajectory to the head controller
        pubHeadcontroller.publish(trajectory)
        
        # Sleep for 0.8 seconds before moving the head again
        rospy.sleep(0.8)
        # Decrement the head movement angle
        headMovement = max(-1, headMovement-0.1)

    rospy.loginfo("Done.")

def moveObejctloop():
    # set a flag to check if an object is found
    objectFound = False
     # loop until objectFound is true
    while objectFound!= True:
        # open json file and read detected object status
        print(objectFound)
        with open(dump_file) as outfile:
            detected = json.load(outfile)
            objectFound = detected['detected']
        # call the movingHead() function
        movingHead()



if __name__ == '__main__':
   
   # Define global variable
    global dump_file
    # Set the dump_file variable with the path of the JSON output file
    dump_file = get_json_file()


    # Open the JSON output file and load the data into the detected variable
    with open(dump_file) as outfile:
        detected = json.load(outfile)

    # Set the 'default' key to False in the detected dictionary
    detected['default'] = False
    # Write the updated detected dictionary back to the JSON output file
    with open(dump_file, 'w') as outfile:
        json.dump(detected, outfile)
    # Initialize the ROS node with the name 'MovementClient'
    rospy.init_node('MovementClient')
    # Create a SimpleActionClient for the '/play_motion' action server
    actionClient = SimpleActionClient('/play_motion', PlayMotionAction)
    # Wait for the action server to become available for a maximum of 20 seconds
    if not actionClient.wait_for_server(rospy.Duration(20)):
        # If the connection times out, log an error message and exit
        rospy.logerr("Could not connect to /play_motion")
        exit()
      
      

    # Create a PlayMotionGoal object
    PMG = PlayMotionGoal()
    # Move the robot to the home position
    homePosition()
    
    
    # Create a publisher to control the head movement    
    pubHeadcontroller = rospy.Publisher('/head_controller/command', JointTrajectory, queue_size=1)
    
    
    # Initialise the head movement variable to 0
    headMovement = 0 
    # Start moving the head
    movingHead()
   