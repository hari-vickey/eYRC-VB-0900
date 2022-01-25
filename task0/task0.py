#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import time
import sys

#theta =0
initial_time = time.time()

def pose_callback(pose):
	global velocity
	#theta = pose.theta
def node_turtle_revolve():
	
	rospy.init_node('node_turtle_revolve',anonymous=False)
	publisher = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=10)
	rospy.Subscriber('/turtle1/pose',Pose,pose_callback)
	
	rate = rospy.Rate(10)

	velocity = Twist()
	while not rospy.is_shutdown():
	    
	    velocity.linear.x = 1.08
	    velocity.linear.y = 0
	    velocity.linear.z = 0
	    velocity.angular.x = 0
	    velocity.angular.y = 0
	    velocity.angular.z = 1.08
	    final_time = time.time()
	    time_elapsed= final_time - initial_time	    
	    rospy.loginfo("Moving in a Circle \n%f",time_elapsed)
            publisher.publish(velocity)
	    if (time_elapsed > 6.3):
		
		velocity.linear.x = 0
	        
	        velocity.angular.z = 1.2
	    	rospy.loginfo("Goal Reached")
		publisher.publish(velocity)
	    	sys.exit()

	    rate.sleep()

if __name__ == '__main__':
	try:
	    node_turtle_revolve()
	except rospy.ROSInterruptException:
	    pass
