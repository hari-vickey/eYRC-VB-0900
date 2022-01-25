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
import tf2_ros
import geometry_msgs.msg
from pkg_vb_sim.msg import LogicalCameraImage
from pkg_vb_sim.srv import conveyorBeltPowerMsg, vacuumGripper
# Creating a Class Ur5Perception
class Ur5Perception:
    #Constructor
    def __init__(self):
        # Wait till the models spawned in gazebo
        rospy.sleep(8)
        # Initializing parameters for Rviz
        self._planning_group = "ur5_1_planning_group"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        self._box_name = ''
        self._display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
        self._exectute_trajectory_client = actionlib.SimpleActionClient('execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()
        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        rospy.loginfo('\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo('\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo('\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')
        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')
        self._tfBuffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._tfBuffer)
        self.func_nec()
        self.func_for_sorting()
    # Function to call neccesary parameters
    def func_nec(self):
        #Invoking the required services
        rospy.wait_for_service('/eyrc/vb/ur5_1/activate_vacuum_gripper')
        rospy.wait_for_service('/eyrc/vb/conveyor/set_power')
        self.belt = rospy.ServiceProxy('/eyrc/vb/conveyor/set_power', conveyorBeltPowerMsg)
        self.grip = rospy.ServiceProxy('/eyrc/vb/ur5_1/activate_vacuum_gripper', vacuumGripper)
        # Assigning the color for this task
        self.red = "logical_camera_2_packagen1_frame"
        self.green = "logical_camera_2_packagen2_frame"
        self.blue = "logical_camera_2_packagen3_frame"
        # Assigning the Frames Required
        self.camera_frame = "logical_camera_2_frame"
        self.reference_frame = "ur5_wrist_3_link"
        self.target_frame = ["logical_camera_2_packagen1_frame", "logical_camera_2_packagen2_frame", "logical_camera_2_packagen3_frame"]
        # Initializing the joint angles for home_position
        self.home_joint = [math.radians(6), math.radians(-142), math.radians(-55), math.radians(-75), math.radians(90), math.radians(0)]
        #Setting joint values to follow waypoints and drop the package in bins
        self.way_point = [math.radians(6), math.radians(-86), math.radians(-126), math.radians(-60), math.radians(90), math.radians(0)]
        self.path_for_redbin = [math.radians(-90), math.radians(-122), math.radians(-85), math.radians(-64), math.radians(90), math.radians(0)]
        self.path_for_greenbin = [math.radians(-8), math.radians(-35), math.radians(71), math.radians(-125), math.radians(-87), math.radians(91)]
        self.path_for_bluebin = [math.radians(100), math.radians(-122), math.radians(-85), math.radians(-64), math.radians(90), math.radians(0)]
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
                rospy.sleep(0.2)
                # 0.16 Is offset given to compensate time delay
                if self.val_y <= 0.16:
                    return
            except AttributeError:
                pass
    # Callback Function
    def func_sub_callback(self, msg):
        try:
            model = msg.models[0].type
            if model == "ur5":
                self.val_y = msg.models[1].pose.position.y
                return
            else:
                self.val_y = msg.models[0].pose.position.y
                return
        except IndexError:
            self.val_y = 1
            return
    # Function to print transform
    def func_translation(self, arg_frame_1, arg_frame_2):
        try:
            trans = self._tfBuffer.lookup_transform(arg_frame_1, arg_frame_2, rospy.Time())
            self.cord_x = trans.transform.translation.x
            self.cord_y = trans.transform.translation.y
            self.cord_z = trans.transform.translation.z
            rospy.loginfo("\n" + "Translation: \n" + "x: {} \n".format(self.cord_x) + "y: {} \n".format(self.cord_y) + "z: {} \n".format(self.cord_z))
        #If any exception caught then respective message is printed
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Package does not exist in the frame limit")
            # Delay is added to limit recursion occurence
            rospy.sleep(0.5)
            self.func_translation(arg_frame_1, arg_frame_2)
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
    #Function to set_joint_angles to move ur5 arm
    def set_joint_angles(self, arg_list_joint_angles):
        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)
        self._group.set_joint_value_target(arg_list_joint_angles)
        self._group.plan()
        flag_plan = self._group.go(wait=True)
        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)
        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)
        if flag_plan == True:
            rospy.loginfo('\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
        else:
            rospy.logerr('\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')
        return flag_plan
    # Function to sort the packages
    def func_for_sorting(self):
        # Calling Appropriate Functions to complete the task
        for i in self.target_frame:
            rospy.loginfo('\033[94m' + "Going to Home Position" + '\033[0m')
            thread = threading.Thread(name="worker", target=self.func_logic)
            thread.start()
            self.set_joint_angles(self.home_joint)
            rospy.sleep(0.2)
            self.func_translation(self.reference_frame, i)
            self.target_x = -self.cord_z
            self.target_y = self.cord_x
            self.target_z = -self.cord_y+0.197
            self.ee_cartesian_translation(self.target_x, self.target_y, self.target_z)
            self.grip(True)
            rospy.loginfo('\033[94m' + "Dropping the package in the bin" + '\033[0m')
            if self.red == i:
                self.set_joint_angles(self.path_for_redbin)
            elif self.green == i:
                self.set_joint_angles(self.path_for_greenbin)
            else:
                self.set_joint_angles(self.path_for_bluebin)
            self.grip(False)
        self.set_joint_angles(self.home_joint)
    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class CartesianPath Deleted." + '\033[0m')
#Main Function
def main():
    # Initialize Node
    rospy.init_node('node_robotic_perception')
    # Creating a object
    ur5 = Ur5Perception()
    #Calling a Destructor
    del ur5
if __name__ == '__main__':
    main()
