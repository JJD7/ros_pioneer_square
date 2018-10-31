#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char** argv){

	// required to access ros. If this gives an error, make sure you are running
	// roscore on your machine.
	ros::init(argc,argv,"square_robot");

	//advertise the topic we wish to publish. Roscore will connect us to the
	//other processes that wish to read from this typic
	ros::NodeHandle nh("/");
	ros::Publisher velout = nh.advertise<geometry_msgs::Twist>("/cmd_vel",5);
	ros::Duration(.1).sleep();


	//A helpful way to print debugging information
	ROS_INFO("Up and running. Run gazebo to simulate the robot and RVIZ to see the sensor data.");

	while(ros::ok()){

		//New outgoing message.
		geometry_msgs::Twist msg;
		msg.linear.x=0.4;
		velout.publish(msg);
		ROS_INFO("Moving Straight!!");

		// wait for some time
		ros::Duration(10).sleep();


		// rotate in place by making linear.x = 0 and angular.z to some non-zero value
		// publish new velocity
		// wait for 90 degrees
		msg.linear.x=0.0;
		msg.angular.z=0.76;
		velout.publish(msg);
		ROS_INFO("Turning!!");
		
		// wait for some time
		ros::Duration(1).sleep();

		// repeat
	}
	return 0;
}
