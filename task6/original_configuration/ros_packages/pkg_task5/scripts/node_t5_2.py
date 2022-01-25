#!/usr/bin/env python
# ROS Node - Robotic Perception and manipulation - Ur5#2
'''
This python file runs a ROS-node of name Ur52Node.
This node acts as a Action Server and this node is also an Action
client to ROS-IOT server.
So it will receive and send the goals asynchronously to the
respective nodes
This nodes controls the ur5_2 arm to pick the packages from the
conveyor belt and drops the package in the respective bins
Sorting is done by the goal received from the Action Client
'''
# Importing required modules
import datetime
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
from pkg_task5.msg import ColorMsgAction, ColorMsgResult
from pkg_ros_iot_bridge.msg import msgRosIotAction, msgRosIotGoal, msgRosIotResult

# Creating a Class Ur5Perception
class Ur52Node(object):
    """
    Class Ur52Node
    This class will act as controller for ur5_2 arm to pick the
    packages from the conveyor belt and drops it in the corresponding
    colored bins
    """
    #Constructor
    def __init__(self):
        # Wait till the models are spawned
        rospy.sleep(8)
        # Initialize Action Server
        self._as = actionlib.ActionServer('/action_ur5_arms',
                                          ColorMsgAction,
                                          self.on_goal,
                                          self.on_cancel,
                                          auto_start=False)
        # Initialize Action Client for Iot Ros communication
        self._ac = actionlib.ActionClient('/action_ros_iot',
                                          msgRosIotAction)
        robot_ns = '/ur5_2'
        planning_group = "manipulator"
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
        # Dictionary to Store all the goal handles
        self._goal_handles = {}
        self.goal_no = 0
        # Initializing Variables
        self.val_y = 1
        self.trans_y = 1
        self.pkg_color = ''
        self.first_order = ''
        self.func_nec()
        rospy.loginfo('\033[94m' + "Planning Group: {}".format(planning_frame) + '\033[0m')
        rospy.loginfo('\033[94m' + "End Effector Link: {}".format(eef_link) + '\033[0m')
        rospy.loginfo('\033[94m' + "Group Names: {}".format(group_names) + '\033[0m')
        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')
        # Start the Action Server
        self._as.start()
        rospy.loginfo('\033[94m' + "Server is Stared and ready to receive goals" + '\033[0m')
        # Wait for the ROS-IOT Action Server
        self._ac.wait_for_server()
        rospy.loginfo('\033[94m' + "Action server(Iot) up, send Goals." + '\033[0m')

    # Function to call neccesary parameters
    def func_nec(self):
        """
        This functions purpose to initialize several parameters and
        joint angles to make the arm work properly
        """
        #Invoking the required services
        rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_2')
        rospy.wait_for_service('/eyrc/vb/conveyor/set_power')
        self.grip = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_2', vacuumGripper)
        self.belt = rospy.ServiceProxy('/eyrc/vb/conveyor/set_power', conveyorBeltPowerMsg)
        # Initializing the joint angles for home_position
        self.home_joint = [math.radians(5), math.radians(-148),
                           math.radians(-42), math.radians(-81),
                           math.radians(91), math.radians(0)]
        #Setting joint values to follow waypoints and drop the package in bins
        self.way_point = [math.radians(6), math.radians(-138),
                          math.radians(-43), math.radians(-90),
                          math.radians(90), math.radians(0)]
        self.path_for_redbin = [math.radians(-90), math.radians(-122),
                                math.radians(-85), math.radians(-64),
                                math.radians(90), math.radians(0)]
        self.path_for_yellowbin = [math.radians(-8), math.radians(-35),
                                   math.radians(71), math.radians(-125),
                                   math.radians(-87), math.radians(91)]
        self.path_for_greenbin = [math.radians(100), math.radians(-122),
                                  math.radians(-85), math.radians(-64),
                                  math.radians(90), math.radians(0)]
        rospy.loginfo('\033[94m' + "Going to Home Position" + '\033[0m')
        self.hard_set_joint_angles(self.home_joint, 5)

    # Function to validate incoming goal
    def on_goal(self, goal_handle):
        """
        On receiving a goal from client this function will be executed.
        This function will accept the goals which are appropriate
        """
        goal = goal_handle.get_goal()
        rospy.loginfo("Received new goal from Client")
        rospy.loginfo('\033[94m' + "Goal Received for Client : {}".format(goal) + '\033[0m')
        # Validate incoming goal parameters
        if goal.color == "red" or goal.color == "yellow" or goal.color == "green":
            goal_handle.set_accepted()
            # 'self.process_goal' - is the function pointer which
            #points to a function that will process incoming Goals
            self.process_goal(goal_handle)
        else:
            goal_handle.set_rejected()
            return

    # This function is to process incoming Goals.
    def process_goal(self, goal_handle):
        """
        This function is the process goal which is received from
        action client to sort the packages
        """
        result = ColorMsgResult()
        result.flag_success = True
        goal_id = goal_handle.get_goal_id()
        goal = goal_handle.get_goal()
        # Goal Processing
        self.func_for_sorting(goal.name, goal.color, goal.orderid,
                              goal.city, goal.time, goal.simtime)
        rospy.loginfo("Send goal result to client")
        if result.flag_success is True:
            rospy.loginfo("Succeeded")
            goal_handle.set_succeeded(result)
        else:
            rospy.loginfo("Goal Failed. Aborting.")
            goal_handle.set_aborted(result)
        rospy.loginfo("Goal ID: " + str(goal_id.id) + " Goal Processing Done.")

    def send_goal_iot(self, *args):
        """
        This function will send goals to the IOT server from this Node using
        msgRosIot.Action file
        """
        # Create a Goal Message Object
        goal = msgRosIotGoal()
        # Extracting arguments from variadic function
        goal.sheet_name = args[0]
        goal.package_name = args[1]
        goal.package_color = args[2]
        goal.order_id = args[3]
        goal.city = args[4]
        goal.shipped_status = args[5]
        goal.shipped_time = args[6]
        rospy.loginfo("Goal has been sent to ROS-IOT server")
        goal_handle = self._ac.send_goal(goal, self.on_transition, None)
        return goal_handle

    # Function to get the state of sent Goal
    def on_transition(self, goal_handle):
        """
        This function monitor the states of all the goals which is sent to
        ROS-IOT server
        This function will give the current state of each goal
        """
        result = msgRosIotResult()
        index = 0
        for i in self._goal_handles:
            if self._goal_handles[i] == goal_handle:
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

    # Function to sort the packages
    def func_for_sorting(self, *args):
        """
        This function will use the incoming goal parameters and sort
        the packages to the respective bins
        This function first calls some functions to stop the coveyor belt
        at appropriate position and to apply translation to the arm to
        go to the package.
        This function is also threaded to send goals to IOT server
        asynchronously
        """
        # Extracting arguments from variadic function
        name = args[0]
        color = args[1]
        ordid = args[2]
        city = args[3]
        ordtime = args[4]
        stime = args[5]
        # Calling Appropriate Functions to sort the packages
        self.func_control()
        self.func_translation()
        self.grip(True)
        thread = threading.Thread(name="worker", target=self.func_parallel)
        thread.start()
        if color == "red":
            rospy.loginfo('\033[94m' + "Dropped the package in the red bin" + '\033[0m')
            self.hard_set_joint_angles(self.way_point, 5)
            self.hard_set_joint_angles(self.path_for_redbin, 5)
        elif color == "yellow":
            rospy.loginfo('\033[94m' + "Dropped the package in the yellow bin" + '\033[0m')
            self.hard_set_joint_angles(self.path_for_yellowbin, 5)
        else:
            rospy.loginfo('\033[94m' + "Dropped the package in the green bin" + '\033[0m')
            self.hard_set_joint_angles(self.way_point, 5)
            self.hard_set_joint_angles(self.path_for_greenbin, 5)
        self.grip(False)
        # Updating the status once the arm drops the package in the bin
        status = "YES"
        thread1 = threading.Thread(name="worker",
                                   target=self.push_data,
                                   args=(name, color, ordid,
                                         city, status, ordtime,
                                         stime))
        thread1.start()
        result = ColorMsgResult()
        result.flag_success = True
        rospy.loginfo('\033[94m' + "Going to Home Position" + '\033[0m')
        self.hard_set_joint_angles(self.home_joint, 5)

    # Function to push data to ROS-IOT Server
    def push_data(self, *args):
        """
        This function is to pre-process the goals which is to be sent to IOT
        server using send goal function
        This function has a if statement according to the sheet name to send
        goals to the IOT server
        """# Extracting arguments from variadic function
        # Extracting arguments from variadic function
        name = args[0]
        color = args[1]
        orderid = args[2]
        city = args[3]
        stat = args[4]
        ordertime = args[5]
        stime = args[6]
        sim_time = rospy.get_rostime()
        secs = datetime.timedelta(seconds=sim_time.secs)
        str_time = datetime.datetime.strptime(ordertime, '%Y-%m-%d %H:%M:%S')
        shipped = str_time + (secs - datetime.timedelta(seconds=stime))
        sheet = "OrdersShipped"
        self._goal_handles[self.goal_no] = self.send_goal_iot(sheet,
                                                              name, color,
                                                              orderid, city,
                                                              stat,
                                                              str(shipped))
        self.goal_no += 1

    # Function to get translational Values
    def func_translation(self):
        """
        this function trasnslation subscribes to the logical camera 2
        topic and applies cartesian translation to make the arm
        move to the package
        """
        while not rospy.is_shutdown():
            try:
                rospy.Subscriber("/eyrc/vb/logical_camera_2",
                                 LogicalCameraImage,
                                 self.func_trans_callback)
                if self.trans_y < 1:
                    self.ee_cartesian_translation(0, self.trans_y, 0)
                    return self.trans_y
            except AttributeError:
                pass

    # Callback Function for translation function
    def func_trans_callback(self, msg):
        """
        this is a callback function to get the translational values
        which in turn applied in cartesian translation
        """
        try:
            model0 = msg.models[0].type
            val_y0 = msg.models[0].pose.position.y
            val_y1 = msg.models[1].pose.position.y
            if model0 == "ur5":
                self.trans_y = val_y1/2
            else:
                self.trans_y = val_y0/2
        except IndexError:
            pass

    # Function to control conveyor
    def func_control(self):
        """
        this function controls the conveyor belt to start and stop
        func_detect is called to detect whether any objects present
        in the belt and then it stops the belt.
        """
        self.belt(100)
        self.func_detect()
        self.belt(0)

    # Function to Detect packages
    def func_detect(self):
        """
        this function detects whether there is a package in
        conveyor belt or not using logical camera 2
        It also returns the val_y so that it can be picked by ur5_2 arm
        """
        while not rospy.is_shutdown():
            try:
                rospy.Subscriber("/eyrc/vb/logical_camera_2",
                                 LogicalCameraImage,
                                 self.func_sub_callback)
                # 0.18 Is offset given to compensate time delay
                rospy.sleep(0.2)
                if self.val_y <= 0.18:
                    return self.val_y
            except AttributeError:
                pass

    # Callback Function for detect function
    def func_sub_callback(self, msg):
        """
        this function is a callback for func_detect
        In this function it will check for the packages rather than ur5
        arm , so that the packages are identified by the camera and
        returns the y co-ordinate of the objects
        """
        try:
            model = msg.models[0].type
            if model == "ur5":
                self.val_y = msg.models[1].pose.position.y
            else:
                self.val_y = msg.models[0].pose.position.y
            return self.val_y
        except IndexError:
            self.val_y = 1
            return self.val_y

    # Function to power up the conveyor while sorting
    def func_parallel(self):
        """
        this function to run the conveyor belt parallely while sorting
        the packages
        """
        rospy.sleep(0.3)
        self.belt(100)

    #Function to set_joint_angles to move ur5 arm
    def set_joint_angles(self, arg_list_joint_angles):
        """
        This function to set the joint angles of the ur5_2 arm
        to the required joint angles as mentioned
        """
        self._group.get_current_joint_values()
        self._group.set_joint_value_target(arg_list_joint_angles)
        self._group.plan()
        flag_plan = self._group.go(wait=True)
        self._group.get_current_joint_values()
        lst_joint_values = self._group.get_current_pose().pose
        rospy.loginfo(lst_joint_values)
        return flag_plan

    # Function to set joint angles forcibly
    def hard_set_joint_angles(self, arg_list_joint_angles, arg_max_attempts):
        """
        This function to set the joint angles forcibly of the ur5_2 arm
        In case, If the arm fails to set the joint angles at the first
        attempt
        """
        number_attempts = 0
        flag_success = False
        while number_attempts <= arg_max_attempts and flag_success is False:
            number_attempts += 1
            flag_success = self.set_joint_angles(arg_list_joint_angles)
            rospy.loginfo("Attempt : {}".format(number_attempts))

    # Function for cartesian translation
    def ee_cartesian_translation(self, trans_x, trans_y, trans_z):
        """
        This ee_cartesian_translation function is to move the end effector
        to the packages, the values are extracted using logical camera 2
        """
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
        rospy.loginfo(fraction)
        num_pts = len(plan.joint_trajectory.points)
        if num_pts >= 3:
            del plan.joint_trajectory.points[0]
            del plan.joint_trajectory.points[1]
        # Make the arm follow the Computed Cartesian Path
        self._group.execute(plan)
    #Function to cancel a goal
    @classmethod
    def on_cancel(cls, goal_handle):
        """
        This function is to cancel any incoming goal
        which is requested by the client
        """
        goal_id = goal_handle.get_goal_id()
        rospy.loginfo("Received cancel request for goal " + str(goal_id))

    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur52Node Deleted." + '\033[0m')

#Main Function
def main():
    """
    This is the main method of this code
    This method will Initialize ROS Node,
    Also, create a instance to the Class Ur52Node
    This main function also uses rospy spin to make the callback function
    executes infintely
    """
    # Initialize Node
    rospy.init_node('node_robotic_perception_ur5_2')
    # Creating Instance for the class Ur5Perception
    Ur52Node()
    #Calling a Destructor
    rospy.spin()


if __name__ == '__main__':
    main()
