#! /usr/bin/env python
# Importing all Necessary Packages
import sys
import math
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from pkg_vb_sim.srv import vacuumGripper
import actionlib
global robot
global scene
# Creating a Class Ur5Moveit
class Ur5Moveit:
    # Constructor
    def __init__(self):
        # Initializing a Node
        rospy.init_node('node_eg3_set_joint_angles', anonymous=True)
        # Initializing required Parameters
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
    # Function to wait till the required state is reached
    def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=1):
        box_name = self._box_name
        scene = self._scene
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0
            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = box_name in scene.get_known_object_names()
            # Test if we are in the expected state
            if box_is_attached == is_attached and box_is_known == is_known:
                return True
                # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()
        # If we exited the while loop without returning then we timed out
        return False
    #Function to Add Box in planning Scene
    def add_box(self, timeout=1):
    	# Initializing Variables
        box_name = self._box_name
        scene = self._scene
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "world"
        box_pose.pose.position.x = 0.02
        box_pose.pose.position.y = 0.50
        box_pose.pose.position.z = 1.85
        box_pose.pose.orientation.w = 1.0
        box_name = "box"
        scene.add_box(box_name, box_pose, size=(0.2, 0.2, 0.2))
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)
    #Function to Attach Box in planning scene
    def attach_box(self, timeout=1):
        box_name = "box"
        robot = self._robot
        scene = self._scene
        eef_link = self._eef_link
        grasping_group = 'ur5_1_planning_group'
        touch_links = robot.get_link_names(group=grasping_group)
        scene.attach_box(eef_link, box_name, touch_links=touch_links)
        return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)
    #Function to detach Box in planning scene
    def detach_box(self, timeout=1):
        box_name = self._box_name
        scene = self._scene
        eef_link = self._eef_link
        scene.remove_attached_object(eef_link, name=box_name)
        return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)
    #Function to remove box in planning scene
    def remove_box(self, timeout=1):
        box_name = "box"
        scene = self._scene
        scene.remove_world_object(box_name)
        return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)
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

    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo('\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')
#Defining a Main Function
def main():
    #Creating a object of the class Ur5Moveit
    ur5 = Ur5Moveit()
    # Setting Joint angles
    lst_joint_angles_1 = [math.radians(69), math.radians(-68), math.radians(-10), math.radians(-102), math.radians(-69), math.radians(-180)]
    lst_joint_angles_2 = [math.radians(6), math.radians(-175), math.radians(-11), math.radians(-83), math.radians(86), math.radians(109)]
    lst_joint_angles_3 = [math.radians(0), math.radians(0), math.radians(0), math.radians(0), math.radians(0), math.radians(0)]
    # Calling Appropriate Functions to complete the task
    ur5.add_box()
    rospy.loginfo('\033[94m' + "Position 1" + '\033[0m')
    ur5.set_joint_angles(lst_joint_angles_1)
    ur5.attach_box()
    rospy.wait_for_service('/eyrc/vb/ur5_1/activate_vacuum_gripper')
    grip = rospy.ServiceProxy('/eyrc/vb/ur5_1/activate_vacuum_gripper', vacuumGripper)
    grip(True)
    rospy.loginfo('\033[94m' + "Position 2" + '\033[0m')
    ur5.set_joint_angles(lst_joint_angles_2)
    ur5.detach_box()
    ur5.remove_box()
    grip(False)
    rospy.loginfo('\033[94m' + "Position 3" + '\033[0m')
    ur5.set_joint_angles(lst_joint_angles_3)
    # Calling a Destructor
    del ur5

if __name__ == '__main__':
    main()
