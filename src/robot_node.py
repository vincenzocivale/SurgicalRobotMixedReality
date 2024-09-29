#! /usr/bin/env python

#Include the necessary libraries 
import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
from math import pi

class MyRobot:

    # Default Constructor
    def __init__(self, Group_Name):

        #Initialize the moveit_commander and rospy node
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('node_set_redefined_pose', anonymous=True)

        
        #Instantiate a RobotCommander object. This object is the outer-level interface to the robot
        self._robot = moveit_commander.RobotCommander()
        #Instantiate a PlanningSceneInterface object. This object is an interface to the world surrounding the robot.
        self._scene = moveit_commander.PlanningSceneInterface()

        self.plan = None
        self.target_pose = None
        
        #define the movegoup for the robotic 
        #Replace this value with your robots planning group name that you had set in Movit Setup Assistant
        self._planning_group = Group_Name
        #Instantiate a MoveGroupCommander Object. This Object is an interface to the one group of joints. this interface can be used to plan and execute the motions on the robotic arm
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        
        #We create a DisplayTrajectory publisher which is used later to publish trajectories for RViz to visualize
        self._display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        #Create action client for the Execute Trajectory action server
        self._exectute_trajectory_client = actionlib.SimpleActionClient('execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        #Get the planning frame, end effector link and the robot group names
        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()

        #print the info
        #here the '\033[95m' represents the standard colour "LightMagenta" in terminals. For details, refer: https://pkg.go.dev/github.com/whitedevops/colors
        #The '\033[0m' is added at the end of string to reset the terminal colours to default
        rospy.loginfo('\033[95m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo('\033[95m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo('\033[95m' + "Group Names: {}".format(self._group_names) + '\033[0m')
        rospy.loginfo('\033[95m' + " >>> MyRobot initialization is done." + '\033[0m')

    def set_pose(self):
        #Set the target pose for the end effector
        self._group.set_pose_target(self.target_pose)
        #Plan the trajectory to the target pose
        plan = self._group.plan()
        #Execute the planned trajectory
        self.plan = plan
    
    def execute_plan(self):
        #Execute the planned trajectory
        self._group.execute(self.plan)
        #Wait for the trajectory to complete
        self._group.get_move_action().wait_for_result()
        #Check the status of the execution
        result = self._group.get_move_action().get_result()
        #Return the result
        return result
    
    def update_target_pose(self):

        target_topic = '/target_pose'

        #Update the target pose
        self.target_pose = rospy.wait_for_message(target_topic, geometry_msgs.msg.PoseStamped)

        
    

    # Class Destructor
    def __del__(self):
        #When the actions are finished, shut down the moveit commander
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[95m' + "Object of class MyRobot Deleted." + '\033[0m')





