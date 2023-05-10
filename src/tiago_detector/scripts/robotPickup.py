#!/usr/bin/python3
import rospy
import sys
import tf
import moveit_commander
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import Pose, PoseStamped

class robotGrasp:
    def __init__(self):
        # Create a listener for TF frames
        self.listener = tf.TransformListener()

        # Create a RobotCommander instance to get information about the robot
        self.robot = moveit_commander.RobotCommander()

        # Create a PlanningSceneInterface instance to interact with the robot's world
        self.scene = moveit_commander.PlanningSceneInterface()

        # Create a MoveGroupCommander instance for the arm_torso group
        self.move_group = moveit_commander.MoveGroupCommander("arm_torso")

        # Set the end effector link for the MoveGroupCommander
        self.move_group.set_end_effector_link("gripper_grasping_frame")

        # Set the goal tolerance for the MoveGroupCommander
        self.move_group.set_goal_tolerance(0.05)

        # Subscribe to the "/objectCentre" topic to receive PoseStamped messages
        # that contain the position and orientation of the object to grasp
        rospy.Subscriber("objectCentre", PoseStamped, self.callback, queue_size=1)

        # Initialize translation and rotation variables to zero
        self.translation = [0, 0, 0]
        self.rotation = [0, 0, 0, 0]

            
    def newPose(self):
        # Create a new Pose object
        newPose = Pose()

        # Set the position coordinates of the pose object based on the current translation values
        newPose.position.x = self.translation[0]
        newPose.position.y = self.translation[1]
        newPose.position.z = self.translation[2]

        # Set the orientation coordinates of the pose object based on the current rotation values
        newPose.orientation.x = self.rotation[0]
        newPose.orientation.y = self.rotation[1]
        newPose.orientation.z = self.rotation[2]
        newPose.orientation.w = self.rotation[3]

        # Return the new Pose object
        return newPose



    def frontPosition(self):

        # Get the current pose of the end-effector
        pose = self.newPose()

        # Move the end-effector 5 cm in front of the current position
        pose.position.x = self.translation[0] - 0.05
        
        return pose


    def callback(self, data):
        # Convert the pose to base frame
        self.poseTobase(data)
        
        # Create a list of waypoints using the current pose
        waypoints = [data.pose]
        
        # Find a plan to the nearest waypoint
        (plan, threshold) = self.findWaypoint(waypoints)

        # Execute the plan if the threshold is within tolerance
        self.planExecution(plan, threshold)


    def poseTobase(self, pose_stamped):
        # Transforms the given pose_stamped from its frame to "/base_footprint" frame.
        poseTobase = self.listener.transformPose("/base_footprint", pose_stamped)

        # Extracts the position and orientation from the transformed pose.
        trans = poseTobase.pose.position
        rot = poseTobase.pose.orientation

        # Saves the position and orientation as attributes of the class.
        self.trans = [trans.x, trans.y, trans.z]
        self.rot = [rot.x, rot.y, rot.z, rot.w]

    def displayInfo(self, print_robot_state=False):
        # Get the name of the planning frame used by MoveIt
        planningFrame = self.move_group.get_planning_frame()
        print(f"Planning Frame: {planningFrame}")

        # Get the name of the end effector link used by MoveIt
        endElink = self.move_group.get_end_effector_link()
        print(f"End effector link: {endElink}")

        # Get the names of all the planning groups available for the robot
        groupNames = self.robot.get_group_names()
        print(f"Available Planning Groups: {groupNames}")

        # If the flag to print the robot's current state is set to True, print it out
        if (print_robot_state):
            print("=== Printing Robot State ===")
            print(self.robot.get_current_state())
            print("=== Stopped Printing Robot State ===")



    # This function plans a cartesian path using the move_group object and a set of waypoints.

    def findWaypoint(self, waypoints):
        # Print a message indicating that the planning process is starting.
        print("Planning Waypoints...")

        # Print the waypoints that will be used for the path planning.
        print(waypoints)

        # Compute the cartesian path using the move_group object.
        # The arguments passed are:
        # - waypoints: a list of Pose objects
        # - eef_step: the step distance used in cartesian path planning, in meters
        # - jump_threshold: the maximum distance in meters allowed between consecutive points in the computed path
        # - avoid_collisions: a boolean flag indicating whether or not to avoid collisions during planning
        (plan, threshold) = self.move_group.compute_cartesian_path(waypoints, 0.02, 0, avoid_collisions=False)

        # Print a message indicating that the planning process has completed and display the threshold value used in planning.
        print("Plan Found")
        print(f"threshold:  {threshold}")

        # Return the computed plan and the threshold value.
        return plan, threshold

    def planExecution(self, plan, threshold):
        # Check if threshold is 1.0, which indicates that the plan is feasible and can be executed
        if threshold == 1.0:
            # Execute the plan, wait until completion
            self.move_group.execute(plan, wait=True)

            # Stop the motion of the robot
            self.move_group.stop()

            # Clear the pose targets to prepare for the next motion planning task
            self.move_group.clear_pose_targets()
        else:
            # If the threshold is not 1.0, it means the plan is not feasible and should not be executed
            print("threshold was not 1.0, not executing path")

def main():
    
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("robotGrasp_node")
    robot_controller = robotGrasp()
    robot_controller.displayInfo()
    
    rospy.spin()

if __name__ == "__main__":
    main()