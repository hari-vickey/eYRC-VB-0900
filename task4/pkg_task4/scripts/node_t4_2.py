#!/usr/bin/env python
# ROS Node - Robotic Perception and manipulation
# Importing required modules
import sys
import math
import copy
import threading
import rospy
import actionlib
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from pkg_vb_sim.msg import LogicalCameraImage
from pkg_vb_sim.srv import conveyorBeltPowerMsg, vacuumGripper
from pkg_task4.msg import ColorMsgAction, ColorMsgResult
# Creating a Class Ur5Perception
class Ur52Node:
    #Constructor
    def __init__(self):
        # Wait till the models are spawned
        rospy.sleep(6)
        # Initialize Action Server
        self._as = actionlib.ActionServer('/action_ur5_arms', ColorMsgAction, self.on_goal, self.on_cancel, auto_start=False)
        self._robot_ns = '/ur5_2'
        self._planning_group = "manipulator"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander(robot_description=self._robot_ns + "/robot_description", ns=self._robot_ns)
        self._scene = moveit_commander.PlanningSceneInterface(ns=self._robot_ns)
        self._group = moveit_commander.MoveGroupCommander(self._planning_group, robot_description=self._robot_ns + "/robot_description", ns=self._robot_ns)
        self._display_trajectory_publisher = rospy.Publisher(self._robot_ns + '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
        self._exectute_trajectory_client = actionlib.SimpleActionClient(self._robot_ns + '/execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()
        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        # Initializing Variables
        self.val_y = 1
        self.pkg_color = ''
        self.func_nec()
        rospy.loginfo('\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo('\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo('\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')
        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')
        # Start the Action Server
        self._as.start()
        rospy.loginfo("Server is Stared and ready to receive goals")
    # Function to call neccesary parameters
    def func_nec(self):
        #Invoking the required services
        rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_2')
        rospy.wait_for_service('/eyrc/vb/conveyor/set_power')
        self.grip = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_2', vacuumGripper)
        self.belt = rospy.ServiceProxy('/eyrc/vb/conveyor/set_power', conveyorBeltPowerMsg)
        # Initializing the joint angles for home_position
        self.home_joint = [math.radians(5), math.radians(-148), math.radians(-42), math.radians(-81), math.radians(91), math.radians(0)]
        #Setting joint values to follow waypoints and drop the package in bins
        self.way_point = [math.radians(6), math.radians(-138), math.radians(-43), math.radians(-90), math.radians(90), math.radians(0)]
        self.path_for_redbin = [math.radians(-90), math.radians(-122), math.radians(-85), math.radians(-64), math.radians(90), math.radians(0)]
        self.path_for_yellowbin = [math.radians(-8), math.radians(-35), math.radians(71), math.radians(-125), math.radians(-87), math.radians(91)]
        self.path_for_greenbin = [math.radians(100), math.radians(-122), math.radians(-85), math.radians(-64), math.radians(90), math.radians(0)]
        '''
        #Setting joint values to drop the packages in respective bins
        self.path_for_redbin = [math.radians(72), math.radians(-47), math.radians(96), math.radians(-139), math.radians(-91), math.radians(-17)]
        self.path_for_yellowbin = [math.radians(-8), math.radians(-35), math.radians(71), math.radians(-125), math.radians(-87), math.radians(91)]
        self.path_for_greenbin = [math.radians(-95), math.radians(-47), math.radians(96), math.radians(-139), math.radians(-91), math.radians(-12)]
        '''
    # Function to validate incoming goal
    def on_goal(self, goal_handle):
        goal = goal_handle.get_goal()
        rospy.loginfo("Received new goal from Client")
        print goal
        # Validate incoming goal parameters
        if goal.color == "red" or goal.color == "yellow" or goal.color == "green":
            goal_handle.set_accepted()
            # 'self.process_goal' - is the function pointer which points to a function that will process incoming Goals
            self.process_goal(goal_handle)
        else:
            goal_handle.set_rejected()
            return
    # This function is to process incoming Goals.
    def process_goal(self, goal_handle):
        result = ColorMsgResult()
        result.flag_success = True
        goal_id = goal_handle.get_goal_id()
        goal = goal_handle.get_goal()
        # Goal Processing
        self.func_for_sorting(goal.color)
        rospy.loginfo("Send goal result to client")
        if result.flag_success == True:
            rospy.loginfo("Succeeded")
            goal_handle.set_succeeded(result)
        else:
            rospy.loginfo("Goal Failed. Aborting.")
            goal_handle.set_aborted(result)
        rospy.loginfo("Goal ID: " + str(goal_id.id) + " Goal Processing Done.")
    # To cancel Goal requests
    def on_cancel(self, goal_handle):
        rospy.loginfo("Received cancel request.")
        goal_id = goal_handle.get_goal_id()
        rospy.loginfo("Goal ID: " + str(goal_id.id) + " Goal has been Cancelled.")
    # Function to get translational Values
    def func_translation(self):
        while not rospy.is_shutdown():
            try:
                rospy.Subscriber("/eyrc/vb/logical_camera_2", LogicalCameraImage, self.func_trans_callback)
                if self.y >= 0 or self.y <= 0:
                    print self.y
                    self.ee_cartesian_translation(0, self.y, 0)
                    return
            except AttributeError:
                pass
    # Callback Function for translation function
    def func_trans_callback(self, msg):
        try:
            model0 = msg.models[0].type
            y0 = msg.models[0].pose.position.y
            y1 = msg.models[1].pose.position.y
            if model0 == "ur5":
                self.y = y1/2
            else:
                self.y = y0/2
        except IndexError:
            pass
    # Function to control conveyor
    def func_logic(self):
        self.belt(100)
        self.func_detect()
        self.belt(0)
    # Function to Detect packages
    def func_detect(self):
        while not rospy.is_shutdown():
            try:
                rospy.Subscriber("/eyrc/vb/logical_camera_2", LogicalCameraImage, self.func_sub_callback)
                # 0.20 Is offset given to compensate time delay
                rospy.sleep(0.2)
                if self.val_y <= 0.18:
                    return
            except AttributeError:
                pass
    # Callback Function for detect function
    def func_sub_callback(self, msg):
        try:
            model = msg.models[0].type
            if model == "ur5":
                self.val_y = msg.models[1].pose.position.y
            else:
                self.val_y = msg.models[0].pose.position.y
        except IndexError:
            self.val_y = 1

    # Function to power up the conveyor while sorting
    def func_parallel(self):
        rospy.sleep(0.3)
        self.belt(100)
    #Function to set_joint_angles to move ur5 arm
    def set_joint_angles(self, arg_list_joint_angles):
        self._group.get_current_joint_values()
        self._group.set_joint_value_target(arg_list_joint_angles)
        self._group.plan()
        flag_plan = self._group.go(wait=True)
        self._group.get_current_joint_values()
        self._group.get_current_pose().pose
        if flag_plan == True:
            rospy.loginfo('\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
        else:
            rospy.logerr('\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')
        return flag_plan
    # Function to set joint angles forcibly
    def hard_set_joint_angles(self, arg_list_joint_angles, arg_max_attempts):
        number_attempts = 0
        flag_success = False
        while number_attempts <= arg_max_attempts and flag_success is False:
            number_attempts += 1
            flag_success = self.set_joint_angles(arg_list_joint_angles)
            rospy.loginfo("Moving to destination. Attempt : {}".format(number_attempts))
    # Function for cartesian translation
    def ee_cartesian_translation(self, trans_x, trans_y, trans_z):
        # Create a empty list to hold waypoints
        waypoints = []
        # Add Current Pose to the list of waypoints
        waypoints.append(self._group.get_current_pose().pose)
        # Create a New waypoint
        wpose = geometry_msgs.msg.Pose()
        wpose.position.x = waypoints[0].position.x + (trans_x)
        wpose.position.y = waypoints[0].position.y + (trans_y)
        wpose.position.z = waypoints[0].position.z + (trans_z)
        # This to keep EE parallel to Ground Plane
        wpose.orientation.x = -0.5
        wpose.orientation.y = -0.5
        wpose.orientation.z = 0.5
        wpose.orientation.w = 0.5
        # Add the new waypoint to the list of waypoints
        waypoints.append(copy.deepcopy(wpose))
        # Compute Cartesian Path connecting the waypoints in the list of waypoints
        (plan, fraction) = self._group.compute_cartesian_path(
            waypoints, 0.01, 0.0)
        rospy.loginfo("Path computed successfully. Moving the arm.")
        num_pts = len(plan.joint_trajectory.points)
        if num_pts >= 3:
            del plan.joint_trajectory.points[0]
            del plan.joint_trajectory.points[1]
        # Make the arm follow the Computed Cartesian Path
        self._group.execute(plan)
    # Function to sort the packages
    def func_for_sorting(self, color):
        # Calling Appropriate Functions to complete the task
        rospy.loginfo('\033[94m' + "Going to Home Position" + '\033[0m')
        self.hard_set_joint_angles(self.home_joint, 5)
        self.func_logic()
        self.func_translation()
        self.grip(True)
        thread = threading.Thread(name="worker", target=self.func_parallel)
        thread.start()
        if color == "red":
            rospy.loginfo('\033[94m' + "Droped the package in the red bin" + '\033[0m')
            self.hard_set_joint_angles(self.way_point, 5)
            self.hard_set_joint_angles(self.path_for_redbin, 5)
        elif color == "yellow":
            rospy.loginfo('\033[94m' + "Droped the package in the yellow bin" + '\033[0m')
            self.hard_set_joint_angles(self.path_for_yellowbin, 5)
        else:
            rospy.loginfo('\033[94m' + "Droped the package in the green bin" + '\033[0m')
            self.hard_set_joint_angles(self.way_point, 5)
            self.hard_set_joint_angles(self.path_for_greenbin, 5)
        self.grip(False)
        self.hard_set_joint_angles(self.home_joint, 5)
        result = ColorMsgResult()
        result.flag_success = True
    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class CartesianPath Deleted." + '\033[0m')
#Main Function
def main():
    # Initialize Node
    rospy.init_node('node_robotic_perception_ur5_2')
    # Creating Instance for the class Ur5Perception
    Ur52Node()
    #Calling a Destructor
    rospy.spin()
if __name__ == '__main__':
    main()
