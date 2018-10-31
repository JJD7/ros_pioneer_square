#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

if __name__ == '__main__':
    #required to access ros. If this gives an error, make sure you are running roscore on your machine.
    rospy.init_node('square_robot')
    #advertise the topic we wish to publish. Roscore will connect us to the other processes that wish to read from this topic
    p = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
    #A helpful way to print debugging information
    rospy.loginfo("Up and running. Run Gazebo to simulate the robot and RVIZ to see the sensor data.")
    while not rospy.is_shutdown():
        twist=Twist()
        twist.linear.x=0.1
        p.publish(twist)
        rospy.loginfo("MOVING STRAIGHT!!")
        # wait for some time
        rospy.sleep(5)


        # rotate in place by making linear.x = 0 and angular.z to some non-zero value
        # publish new velocity
        twist.linear.x=0.0
        twist.angular.z=0.78
        p.publish(twist)
        rospy.loginfo("Turning!!")
        
	# wait for 90 degrees
        rospy.sleep(2)
        
	# repeat




