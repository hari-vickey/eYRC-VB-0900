#! /usr/bin/env python
# ROS Node - Robotic Perception and Manipulation - Ur5#1
# Importing Required Modules
import sys
import threading
import rospy
import actionlib
import rospkg
import yaml
import moveit_commander
import moveit_msgs.msg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from pyzbar.pyzbar import decode
from pkg_vb_sim.srv import vacuumGripper
from pkg_task4.msg import ColorMsgAction, ColorMsgGoal, ColorMsgResult
import cv2
# Created a Class Camera1
class Camera1:
    # Constructor
    def __init__(self):
        self.bridge = CvBridge()
        # Intializing List for storing colors detected by 2D camera
        self.pkgname_lst = []
        self.color_lst = []
        self.lst = []
        self.flag = 0
        # Call the function camera to extract color
        self.camera()
    # Function for camera
    def camera(self):
        while not rospy.is_shutdown():
            self.image_sub = rospy.Subscriber("/eyrc/vb/camera_1/image_raw", Image, self.callback)
            if self.flag == 1:
                print '\033[94m' + "Color of packages extracted from 2D camera in list form" + '\033[0m'
                print self.lst
                return
    # CallBack Function for subscriber
    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as err_msg:
            rospy.logerr(err_msg)
        # Cleaning the image
        crop_img = cv_image[287:904, 72:648]
        gray = cv2.cvtColor(crop_img, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 25, 255, cv2.THRESH_BINARY)
        # Calling Detect Function
        codes = ''
        codes = decode(thresh)
        # Declaring constants for re arranging the detected QR_Code
        code1 = []
        code2 = []
        pkg = ["packagen00", "packagen01", "packagen02", "packagen10", "packagen11", "packagen12", "packagen20", "packagen21", "packagen22", "packagen30", "packagen31", "packagen32"]
        self.pkgname_lst = pkg
        i = j = 0
        k = 3
        # Rearrange the detected QR_Code by top order
        while codes:
            min_top = codes[0].rect.top
            for item in codes:
                if item.rect.top <= min_top:
                    min_top = item.rect.top
                    new = item
            code1.append(new)
            codes.remove(new)
        # Rearrange the detected QR_Code by left order
        while code1:
            try:
                min_left = code1[0].rect.left
                while i < k:
                    if code1[i].rect.left <= min_left:
                        min_left = code1[i].rect.left
                        new1 = code1[i]
                    i += 1
                k -= 1
                i = 0
                code2.append(new1)
                code1.remove(new1)
                if k < 1:
                    k = 3
            except IndexError:
                code2 = code1
                break
        # Deleting contents of the list
        del self.color_lst[:]
        del self.lst[:]
        # Mapping the detected QR_Code
        for code in code2:
            data = code.data
            txt = pkg[j] +'-'+ data
            self.lst.append(txt)
            self.color_lst.append(data)
            j += 1
        self.flag = 1
# Creating a class Ur5_1_Node
class Ur51Node:
    #Constructor
    def __init__(self):
        # Wait till all the models are spawned in gazebo
        rospy.sleep(6)
        # Initializing Action server
        self._ac = actionlib.ActionClient('/action_ur5_arms', ColorMsgAction)
        # Dictionary to Store all the goal handels
        self._goal_handles = {}
        # Initializing Parameters For Ur5#1
        self._robot_ns = '/ur5_1'
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
        self._computed_plan = ''
        rp = rospkg.RosPack()
        self._pkg_path = rp.get_path('pkg_task4')
        self._file_path = self._pkg_path + '/config/saved_trajectories/'
        rospy.loginfo("Package Path: {}".format(self._file_path))
        rospy.loginfo('\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo('\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo('\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')
        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')
        #Invoking the required services
        rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1')
        self.grip = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1', vacuumGripper)
        # Implemeting threading
        thread = thread = threading.Thread(name="worker", target=self.func_store)
        thread.start()
        # Wait For Server
        self._ac.wait_for_server()
        rospy.loginfo("Action server(#ur5_2) is up, we can send new goals!")
    # Function to store the color of the package detected in Camera Class
    def func_store(self):
        ic = Camera1()
        self.package_name = ic.pkgname_lst
        self.package_color = ic.color_lst
        # Initializing the home point
        self.moveit_hard_play_planned_path_from_file(self._file_path, 'home_point_1.yaml', 5)
        self.func_task()
    # Function to send goals to Action Server
    def send_goal(self, arg_name, arg_color):
        # Create a Goal Message Object
        goal = ColorMsgGoal()
        goal.name = arg_name
        goal.color = arg_color
        rospy.loginfo("Goal has been sent.")
        goal_handle = self._ac.send_goal(goal, self.on_transition, None)
        return goal_handle
    # Function to get the state of sent Goal
    def on_transition(self, goal_handle):
        result = ColorMsgResult()
        index = 0
        for i in self._goal_handles:
            if self._goal_handles[i] == goal_handle:
                index = i + 1
                break
        rospy.loginfo("Transition Callback. Client Goal Handle #: " + str(index))
        # Comm State - Monitors the State Machine of the Client which is different from Server's
        # Comm State = 2 -> Active
        # Comm State = 3 -> Wating for Result
        # Comm State = 7 -> Done
        if goal_handle.get_comm_state() == 2:
            rospy.loginfo(str(index) + ": Goal just went active.")
        if goal_handle.get_comm_state() == 7:
            rospy.loginfo(str(index) + ": Goal is DONE")
            rospy.loginfo(goal_handle.get_terminal_state())
            result = goal_handle.get_result()
            rospy.loginfo(result.flag_success)
            if result.flag_success == True:
                rospy.loginfo("Goal successfully completed. Client Goal Handle #: " + str(index))
            else:
                rospy.loginfo("Goal failed. Client Goal Handle #: " + str(index))
    # Function to complete the task
    def func_task(self):
        for i in range(0, 9):
            self._goal_handles[i] = self.send_goal(self.package_name[i], self.package_color[i])
            self.pick_and_place(self.package_name[i])
    # Function to pick and place the package in conveyor
    def pick_and_place(self, pkg):
        pick = pkg + '.yaml'
        inter = pkg + 'I' + '.yaml'
        drop = pkg + 'D' + '.yaml'
        self.moveit_hard_play_planned_path_from_file(self._file_path, pick, 5)
        self.grip(True)
        self.moveit_hard_play_planned_path_from_file(self._file_path, inter, 5)
        self.moveit_hard_play_planned_path_from_file(self._file_path, drop, 5)
        self.grip(False)
    # Function to play planned path
    def moveit_play_planned_path_from_file(self, arg_file_path, arg_file_name):
        file_path = arg_file_path + arg_file_name

        with open(file_path, 'r') as file_open:
            loaded_plan = yaml.load(file_open)

        ret = self._group.execute(loaded_plan)
        return ret
    # Function to Hard play planned path
    def moveit_hard_play_planned_path_from_file(self, arg_file_path, arg_file_name, arg_max_attempts):
        number_attempts = 0
        flag_success = False

        while ((number_attempts <= arg_max_attempts) and (flag_success is False)):
            number_attempts += 1
            flag_success = self.moveit_play_planned_path_from_file(arg_file_path, arg_file_name)
            rospy.loginfo("Moving to destination. Attempt : {}".format(number_attempts))
        return True
    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class CartesianPath Deleted." + '\033[0m')
#Main Function
def main():
    # Initialize Node
    rospy.init_node('node_robotic_perception_ur5_1')
    # Creating an Instance to Class ur5_1
    Ur51Node()
    # DO Not Exit and loop forever
    rospy.spin()
if __name__ == '__main__':
    main()
