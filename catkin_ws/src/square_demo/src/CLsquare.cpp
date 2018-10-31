#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include "nav_msgs/Odometry.h"
#include <std_msgs/Float32.h>
#include <math.h>
#include <string>

class RobotClass{

	public:
	RobotClass(ros::NodeHandle* nodehandle);

	private:
	ros::NodeHandle nh_;
	ros::Subscriber odom_info;
	ros::Publisher velout;
	geometry_msgs::Twist output_msg;
	void initializeSubscribers();
	void initializePublishers();
	void controlLoop();
	void getState();

	void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

	float x_pos;
	float y_pos;
	float rotation;
	float lin_speed;
	float rot_speed;
	float current_err;
	int state_val;
	bool settled;
	bool check_status;
};


RobotClass::RobotClass(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{//contructor

	x_pos = 0.0;
	y_pos = 0.0;
	rotation = 0.0;
	current_err = 0;
	lin_speed = 0.4;
	rot_speed = 0.2;
	state_val = 0;
	settled = false;
	check_status = false;

	ROS_INFO("Preparing to initialize");
	initializeSubscribers();
	initializePublishers();

}

void RobotClass::initializeSubscribers()
{
    ROS_INFO("Initializing Subscribers");
    odom_info = nh_.subscribe("odom", 10, &RobotClass::odomCallback,this);

}

void RobotClass::initializePublishers()
{
    ROS_INFO("Initializing Publishers");
    velout = nh_.advertise<geometry_msgs::Twist>("/cmd_vel",5);;
		ROS_INFO("Moving... The Current Control State is %i", state_val);
}


void RobotClass::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
//Most of the heavy lifting is done from this callback.
//The pose is read from the odometry message and indexed for pose info I am using to control the robot.
    x_pos = msg->pose.pose.position.x;
		y_pos = msg->pose.pose.position.y;
		rotation = msg->pose.pose.orientation.z;

		//A control loop is initiated which executes the control signals
		//The control signals depend on the state which is updated when the current state goal is reached.
		controlLoop();
    getState();

    velout.publish(output_msg); //output the desired control signal;
}

void RobotClass::controlLoop() {

	if(settled){//stop motion and wait
		output_msg.linear.x = 0.0;
		output_msg.linear.y = 0.0;
		output_msg.linear.z = 0.0;
		output_msg.angular.x = 0.0;
		output_msg.angular.y = 0.0;
		output_msg.angular.z = 0.0;
		velout.publish(output_msg);	//send stop values now
		ros::Duration(1).sleep(); //give some time for the robot to stop moving.
		settled = false; // need to set false so that next state will start.
		check_status = false;
		ROS_INFO("Moving... The Current Control State is %i", state_val);
	}
	else{
//State machine to drive robot to different goals
		switch(state_val){
			case 0: //move x till position x=4
				current_err = (4-x_pos);
				output_msg.linear.x = lin_speed;
				output_msg.angular.z = 0.0;
				break;
			case 1:
				current_err = (0.7071-rotation);
				output_msg.linear.x = 0.0;
				output_msg.angular.z = rot_speed;
				break;
			case 2://move x till position y=4
				current_err = (4-y_pos);
				output_msg.linear.x = lin_speed;
				output_msg.angular.z = 0.0;
				break;
			case 3://rotate another 90 degrees
				current_err = (1-rotation);
				output_msg.linear.x = 0.0;
				output_msg.angular.z = rot_speed;
				break;
			case 4:// move back to position x=0
				current_err = (x_pos-0);
				output_msg.linear.x = lin_speed;
				output_msg.angular.z = 0.0;
				break;
			case 5:
				current_err = (rotation-0.7071);
				output_msg.linear.x = 0.0;
				output_msg.angular.z = rot_speed;
				break;
			case 6://move back to position y=0
				current_err = (y_pos-0);
				output_msg.linear.x = lin_speed;
				output_msg.angular.z = 0.0;
				break;
			case 7:
				current_err = (rotation-0);
				output_msg.linear.x = 0.0;
				output_msg.angular.z = rot_speed;
				break;
		}

		check_status = true; //want to check settling conditions after execution
	}


}

void RobotClass::getState(){
	if (check_status && current_err < 0.0001){ //settled around setpoint with defined precission
		//need to clear error for next goal
		current_err =0.0;

		//tell control loop not to continue running
		settled = true;
		//get next state
		if(state_val < 7){
			++state_val;
			ROS_INFO("Goal Reached. Stopping motion...");
		}
		else{ //reset state to 0
			state_val = 0;
			ROS_INFO("Goal Reached. Stopping motion...");
		}
	}
	else{
		settled = false;
	}
}

int main(int argc, char **argv){


	// required to access ros. If this gives an error, make sure you are running
	// roscore on your machine.
	ros::init(argc,argv,"square_robot");
	ros::NodeHandle nh;
	ros::Duration(.1).sleep();
	ROS_INFO("Running");

	RobotClass robot(&nh);	//instantiate a RobotClass object and pass pointer to nodehandle for constructor to use

	while(ros::ok()){

		ros::spinOnce();

	}
	return 0;
}
