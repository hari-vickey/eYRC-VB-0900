#! /usr/bin/env python
# ROS Node - Robotic Perception and Manipulation - Ur5#1
'''
This python file runs a ROS-node of name Ur51Node.
This node acts as a Action client for both Ur5_2 server and ROS-IOT server.
So it will send the goals asynchronously to both servers
This Node contains two classes one for the 2d camera and ur5_1
Class 2d camera detects the package color and name
Class ur5_1 makes the arm to pick and place the package
'''
# Importing Required Modules
import threading
import datetime
import json
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
from pkg_task5.msg import ColorMsgAction, ColorMsgGoal, ColorMsgResult
from pkg_ros_iot_bridge.msg import msgRosIotAction, msgRosIotGoal, msgRosIotResult
from pkg_ros_iot_bridge.msg import msgMqttSub
import cv2

# Created a Class Camera1
class Camera1(object):
    """
    Class Camera1
    This is the image processing unit of this node.
    This class will detect the package color which is present in shelf
    """
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
        """
        This function is an infinite loop which subscribes to the ROS topic
        of 2d camera to get the input
        """
        while not rospy.is_shutdown():
            self.image_sub = rospy.Subscriber("/eyrc/vb/camera_1/image_raw", Image, self.callback)
            # If all packages are detected it will return flag as 1
            if self.flag == 1:
                rospy.loginfo('\033[94m' + "Color of packages extracted \
                from 2D camera in list form\n" + '\033[0m' + str(self.lst))
                return self.flag

    # CallBack Function for Camera subscriber
    def callback(self, data):
        """
        This is callback function for camera
        This function will convert the raw image to bgr8 and calls
        qr detect function
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as err_msg:
            rospy.logerr(err_msg)
        image = cv_image
        # Calling Detect Function
        self.qr_detect(image)
        self.flag = 1

    # Function to detect QR_Code
    def qr_detect(self, arg_image):
        """
        This function calls process the images from 2d camera
        It will crop the image, convert color to grayscale and makes
        threshold and decode the qr codes.
        """
        # Cropping the image to required frames
        crop_img = arg_image[287:904, 72:648]
        # Cleaning the image
        gray = cv2.cvtColor(crop_img, cv2.COLOR_BGR2GRAY)
        # Thresholding the image to detect the qr codes
        _, thresh = cv2.threshold(gray, 25, 255, cv2.THRESH_BINARY)
        coded = decode(thresh)
        # If all the packages are not detected, then adaptive
        # thresholding is implemented
        if len(coded) < 12:
            _, thresh1 = cv2.threshold(gray, 20, 255, cv2.THRESH_BINARY)
            code1 = decode(thresh1)
            if len(code1) == 12:
                self.rearrange_img(code1)
            else:
                thresh2 = cv2.adaptiveThreshold(gray, 255,
                                                cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                                cv2.THRESH_BINARY, 11, 2)
                code2 = decode(thresh2)
                if len(code2) == 12:
                    self.rearrange_img(code2)
                else:
                    thresh3 = cv2.adaptiveThreshold(gray, 255,
                                                    cv2.ADAPTIVE_THRESH_MEAN_C,
                                                    cv2.THRESH_BINARY, 11, 2)
                    code3 = decode(thresh3)
                    self.rearrange_img(code3)
        else:
            self.rearrange_img(coded)

    # Function to rearrange Image codes
    def rearrange_img(self, codes):
        """
        This function re-arranges the decoded data
        By using the decoded data Re-arrangement is made to get
        decoded package color in order.
        Atlast the decoded colors and package name are stored in list.
        """
        # Declaring constants for re arranging the detected QR_Code
        code1 = []
        code2 = []
        pkg = ["packagen00", "packagen01", "packagen02",
               "packagen10", "packagen11", "packagen12",
               "packagen20", "packagen21", "packagen22",
               "packagen30", "packagen31", "packagen32"]
        pkg_color = []
        self.pkgname_lst = pkg
        i = 0
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
        del pkg_color[:]
        del self.lst[:]
        # Mapping the detected QR_Code
        i = 0
        for code in code2:
            txt = pkg[i] +'-'+ code.data
            self.lst.append(txt)
            pkg_color.append(code.data)
            i += 1
        self.color_lst = pkg_color
        self.flag = 1

# Creating a class Ur5_1_Node
class Ur51Node(object):
    """
    class Ur51Node
    This class acts as a action client for ur5_2 server and ROS-IOT server
    This class has algorithm to prioritize the package from incomingorders
    This class also do the pick and place operation
    """
    #Constructor
    def __init__(self):
        # Initialize Action Client for Iot Ros communication
        self._ac1 = actionlib.ActionClient('/action_ros_iot', msgRosIotAction)
        # Wait till all the models are spawned in gazebo
        rospy.sleep(8)
        # Initializing Action server for communication between ur5 arms
        self._ac2 = actionlib.ActionClient('/action_ur5_arms', ColorMsgAction)
        # Dictionary to Store all the goal handles
        self._goal_handles1 = {}
        self._goal_handles2 = {}
        # Initializing Parameters For Ur5#1
        robot_ns = '/ur5_1'
        planning_group = "manipulator"
        #self._commander = moveit_commander.roscpp_initialize(sys.argv)
        robot = moveit_commander.RobotCommander(robot_description=
                                                robot_ns +
                                                "/robot_description",
                                                ns=robot_ns)
        moveit_commander.PlanningSceneInterface(ns=robot_ns)
        self._group = moveit_commander.MoveGroupCommander(planning_group,
                                                          robot_description=
                                                          robot_ns +
                                                          "/robot_description",
                                                          ns=robot_ns)
        rospy.Publisher(robot_ns + '/move_group/display_planned_path',
                        moveit_msgs.msg.DisplayTrajectory, queue_size=1)
        exectute_trajectory_client = actionlib.SimpleActionClient(robot_ns +
                                                                  '/execute_trajectory',
                                                                  moveit_msgs.
                                                                  msg.ExecuteTrajectoryAction)
        exectute_trajectory_client.wait_for_server()
        planning_frame = self._group.get_planning_frame()
        eef_link = self._group.get_end_effector_link()
        group_names = robot.get_group_names()
        # Declaring goal no's to know how many goals are sent
        self.goal_no1 = 0
        self.goal_no2 = 0
        # Declaring Constants and lists
        self.orderid = []
        self.city = []
        self.priority_handle = []
        self.priority_handle_color = []
        self.package_name = []
        self.package_color = []
        self.order_time = []
        self.order_simtime = []
        self.flag_complete = False
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('pkg_task5')
        self._file_path = pkg_path + '/config/saved_trajectories/'
        rospy.loginfo("Package Path: {}".format(self._file_path))
        rospy.loginfo('\033[94m' + "Planning Group: {}".format(planning_frame) + '\033[0m')
        rospy.loginfo('\033[94m' + "End Effector Link: {}".format(eef_link) + '\033[0m')
        rospy.loginfo('\033[94m' + "Group Names: {}".format(group_names) + '\033[0m')
        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')
        #Invoking the required services
        rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1')
        self.grip = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1', vacuumGripper)
        # Implemeting threading
        thread = threading.Thread(name="worker", target=self.func_store)
        thread.start()
        self.func_mqtt()
        # Wait for UR5_2 Action Server
        self._ac2.wait_for_server()
        rospy.loginfo("Action server(#ur5_2) is up, we can send new goals!")

    # Function to store the color of the package detected in Camera Class
    def func_store(self):
        """
        This function is store the package name and color which is
        extracted from camera class
        This function is also integrated with a thread which can
        push data to ROS-IOT server asynchronously while making the
        arm to move from allZeros to home_point
        """
        camera = Camera1()
        self.package_name = camera.pkgname_lst
        self.package_color = camera.color_lst
        # Wait for Iot Action Server
        self._ac1.wait_for_server()
        rospy.loginfo("Action server(Iot) up, we can receive orders.")
        thread = threading.Thread(name="worker",
                                  target=self.push_data,
                                  args=("Inventory", self.package_name,
                                        self.package_color,
                                        "NA", "NA", "NA",
                                        "NA", "NA"))
        thread.start()
        # Initializing the home point
        self.moveit_hard_play_planned_path_from_file(self._file_path, 'home_point.yaml', 5)

    # Function to receive orders
    def func_mqtt(self):
        """
        This function subscribes to the topic /ros_iot_bridge/mqtt/sub
        to receive the goals which is published from IOT server
        This function also makes the call func_algorithm
        """
        #Subscribe to the Mqtt Subscription topic
        rospy.Subscriber('/ros_iot_bridge/mqtt/sub', msgMqttSub, self.mqtt_callback)
        # Calling Function algorithm to prioritize goals
        self.func_algorithm(self.priority_handle,
                            self.priority_handle_color,
                            self.orderid, self.city,
                            self.order_time, self.order_simtime)

    # Callback Function for Mqtt Subscription
    def mqtt_callback(self, msg):
        """
        This is a callback function which will execute when order is
        published in ros topic and converts them to required format
        This function will also assign different parameters to various
        lists which is used later on.
        """
        # Convert String to dictionary using json library
        orders = json.loads(msg.message)
        rospy.loginfo('\033[94m' + "Order Received\n" + '\033[0m' + str(orders))
        ros_simtime = rospy.get_rostime()
        simtime = ros_simtime.secs
        time = orders['order_time']
        if orders['item'] == "Medicine":
            order_color = "red"
            weightage = 4
            ordid = orders['order_id']
            ordcity = orders['city']
        elif orders['item'] == "Food":
            order_color = "yellow"
            weightage = 2
            ordid = orders['order_id']
            ordcity = orders['city']
        else:
            order_color = "green"
            weightage = 1
            ordid = orders['order_id']
            ordcity = orders['city']
        # Assigning List to apply in algorithm
        self.priority_handle.append(weightage)
        self.priority_handle_color.append(order_color)
        # Assigning order Id, City and time to a list
        self.orderid.append(ordid)
        self.city.append(ordcity)
        self.order_time.append(time)
        self.order_simtime.append(simtime)

    # Function Algorithm to Prioritize Packages
    def func_algorithm(self, *args):
        """
        This function algorithm will prioritize the packages if there are
        multiple orders to be processed
        This function also deletes the processed elements from the list
        """
        while True:
            # Assigning to a separate variables from the variadic function
            priority = args[0]
            pick_color = args[1]
            orderid = args[2]
            city = args[3]
            time = args[4]
            stime = args[5]
            while priority and self.flag_complete is False:
                rospy.loginfo(priority)
                i = priority.index(max(priority))
                rospy.loginfo("Prioritized Goal is " + pick_color[i] + ' ' + str(priority[i]))
                self.func_task(pick_color[i], orderid[i], city[i], time[i], stime[i])
                # Delete the elements once they are processed.
                del self.priority_handle[i]
                del self.priority_handle_color[i]
                del self.orderid[i]
                del self.city[i]
                del self.order_time[i]
                del self.order_simtime[i]

    # Function to Send Goal to IOT server
    def send_goal_iot(self, *args):
        """
        This function will send goals to the IOT server from this Node using
        msgRosIot.Action file
        """
        # Create a Goal Message Object
        goal = msgRosIotGoal()
        goal.sheet_name = args[0]
        goal.package_name = args[1]
        goal.package_color = args[2]
        goal.dispatch_status = args[3]
        goal.order_id = args[4]
        goal.city = args[5]
        goal.dispatch_time = args[6]
        rospy.loginfo("Goal has been sent to ROS-IOT server")
        goal_handle1 = self._ac1.send_goal(goal, self.on_transition1, None)
        return goal_handle1

    # Function to send goals to Action Server
    def send_goal_ur5_2(self, *args):
        """
        This function will send goals to the Ur5_2 server from this Node using
        ColorMsg.Action file
        """
        # Create a Goal Message Object
        goal = ColorMsgGoal()
        goal.name = args[0]
        goal.color = args[1]
        goal.orderid = args[2]
        goal.city = args[3]
        goal.time = args[4]
        goal.simtime = args[5]
        rospy.loginfo("Goal has been sent to ur5_2 arm.")
        goal_handle2 = self._ac2.send_goal(goal, self.on_transition2, None)
        return goal_handle2

    # Function to get the state of sent Goal
    def on_transition1(self, goal_handle):
        """
        This function monitor the states of all the goals which is sent to
        ROS-IOT server
        This function will give the current state of each goal
        """
        result = msgRosIotResult()
        index = 0
        for i in self._goal_handles1:
            if self._goal_handles1[i] == goal_handle:
                index = i + 1
                break
        rospy.loginfo("TransitionCallback.Client Goal Handle #: " + str(index))
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
            if result.flag_success is True:
                rospy.loginfo("Goal successfully completed. Client Goal Handle #: " + str(index))
            else:
                rospy.loginfo("GoalFailed.Client Goal Handle #: " + str(index))

    # Function to get the state of sent Goal
    def on_transition2(self, goal_handle):
        """
        This function monitor the states of all the goals which is sent to
        Ur5_2 server
        This function will give the current state of each goal
        """
        result = ColorMsgResult()
        index = 0
        for i in self._goal_handles2:
            if self._goal_handles2[i] == goal_handle:
                index = i + 1
                break
        rospy.loginfo("TransitionCallback.Client Goal Handle #: " + str(index))
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
            if result.flag_success is True:
                rospy.loginfo("Goal successfully completed. Client Goal Handle #: " + str(index))
            else:
                rospy.loginfo("GoalFailed.Client Goal Handle #: " + str(index))

    # Function to push data to Iot Action Server
    def push_data(self, *args):
        """
        This function is to pre-process the goals which is to be sent to IOT
        server using send goal function
        This function has a if statement according to the sheet name to send
        goals to the IOT server
        """
        # Assigning to a seperate variables from the variadic function
        sheet = args[0]
        packname = args[1]
        packcolor = args[2]
        stat = args[3]
        orderid = args[4]
        city = args[5]
        time = args[6]
        stime = args[7]
        # if statement to push data properly
        if sheet == "Inventory":
            for i in range(0, 12):
                self._goal_handles1[i] = self.send_goal_iot(sheet,
                                                            packname[i],
                                                            packcolor[i],
                                                            stat,
                                                            orderid,
                                                            city, "NA")
        elif sheet == "OrdersDispatched":
            sim_time = rospy.get_rostime()
            str_time = datetime.datetime.strptime(time, '%Y-%m-%d %H:%M:%S')
            secs = datetime.timedelta(seconds=sim_time.secs)
            dtime = str_time + (secs - datetime.timedelta(seconds=stime))
            self._goal_handles1[self.goal_no1] = self.send_goal_iot(sheet,
                                                                    packname,
                                                                    packcolor,
                                                                    stat,
                                                                    orderid,
                                                                    city,
                                                                    str(dtime))
            self.goal_no1 += 1

    # Function to complete the task
    def func_task(self, *args):
        """
        This function is used to search the prioritized package
        avaliability in shelf and send the goal to ur5_2 server
        Once the specified package is identified then the
        package name and color are removed in the Inventory
        """
        # Assigning to a seperate variables from the variadic function
        color = args[0]
        orderid = args[1]
        city = args[2]
        time = args[3]
        stime = args[4]
        self.flag_complete = True
        i = self.package_color.index(color)
        self._goal_handles2[self.goal_no2] = self.send_goal_ur5_2(self.package_name[i],
                                                                  self.package_color[i],
                                                                  orderid,
                                                                  city,
                                                                  time, stime)
        self.pick_and_place(self.package_name[i], self.package_color[i], orderid, city, time, stime)
        # Deleting the elements which are processed.
        del self.package_color[i]
        del self.package_name[i]
        self.goal_no2 += 1

    # Function to pick and place the package in conveyor
    def pick_and_place(self, *args):
        """
        This function makes the arm to pick and place the packages from the
        shelf to the conveyor
        The pick and place operation uses pre-defined trajectories which is
        saved as yaml files in pkg_task5
        This function also implemented with a thread to push data to
        Iot server
        """
        # Assigning to a seperate variables from the variadic function
        package = args[0]
        color = args[1]
        orderid = args[2]
        city = args[3]
        time = args[4]
        stime = args[5]
        # Creating appropriate name of the file to be used for
        # pick and place
        pick = package + '.yaml'
        inter = package + 'I' + '.yaml'
        drop = package + 'D' + '.yaml'
        self.moveit_hard_play_planned_path_from_file(self._file_path, pick, 5)
        self.grip(True)
        self.moveit_hard_play_planned_path_from_file(self._file_path, inter, 5)
        self.moveit_hard_play_planned_path_from_file(self._file_path, drop, 5)
        self.grip(False)
        thread = threading.Thread(name="worker",
                                  target=self.push_data,
                                  args=("OrdersDispatched",
                                        package, color, "YES",
                                        orderid, city, time,
                                        stime))
        thread.start()
        self.flag_complete = False

    # Function to play planned path
    def moveit_play_planned_path_from_file(self, arg_file_path, arg_file_name):
        """
        This function will play the planned path which is saved in the format
        of yaml file in pkg_task5
        """
        file_path = arg_file_path + arg_file_name
        with open(file_path, 'r') as file_open:
            loaded_plan = yaml.load(file_open, Loader=yaml.Loader)

        ret = self._group.execute(loaded_plan)
        return ret

    # Function to Hard play planned path
    def moveit_hard_play_planned_path_from_file(self, arg_file_path, arg_file_name, arg_max):
        """
        This will play the trajectories multiple times so that
        trajectories will not fail
        """
        attempts = 0
        flag_success = False
        while ((attempts <= arg_max) and (flag_success is False)):
            attempts += 1
            flag_success = self.moveit_play_planned_path_from_file(arg_file_path, arg_file_name)
            rospy.loginfo("Attempt : {}".format(attempts))
        return True

    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur51Node Deleted." + '\033[0m')

#Main Function
def main():
    """
    This is the main function of this code which will initialize the
    ROS node and create a instances of ur51 node
    Rospy spin also used to make the callbacks to execute infinitely
    """
    # Initialize Node
    rospy.init_node('node_robotic_perception_ur5_1')
    # Creating an Instance to Class ur5_1
    Ur51Node()
    # DO Not Exit and loop forever
    rospy.spin()


if __name__ == '__main__':
    main()
