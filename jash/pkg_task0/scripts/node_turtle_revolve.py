#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import time
import math
def main():
	rospy.init_node('node_turtle_revolve', anonymous=True)
	velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
	vel_msg = Twist()
	omega=0.53
	vel_msg.linear.x=0.0
	vel_msg.linear.y=0.0
	vel_msg.linear.z=0.0
	vel_msg.angular.x=0.0	
	vel_msg.angular.y=0.0	
	vel_msg.angular.z=0.0	
	
	theta=0
	start=rospy.Time.now().to_sec()
	rate=rospy.Rate(10)
	while not rospy.is_shutdown():
		end=rospy.Time.now().to_sec()
		interval=end-start
		theta=omega*interval
		if(theta<math.pi*2):	
			vel_msg.linear.x=1.0
			vel_msg.angular.z=0.5
			#print("Theta is %d",theta)
			#print(velocity_publisher)
			rospy.loginfo("Moving in a circle")		
			#print("Moving in a circle")
			print(theta)			
			velocity_publisher.publish(vel_msg)
			rate.sleep()

		else:	
			rospy.loginfo("goal reached")
			vel_msg.linear.x=0.0
			vel_msg.linear.y=0.0
			vel_msg.linear.z=0.0
			vel_msg.angular.x=0.0	
			vel_msg.angular.y=0.0	
			vel_msg.angular.z=0.0	
			break;
		
if __name__ == '__main__':
      try:
           main()
      except rospy.ROSInterruptException:
           pass
