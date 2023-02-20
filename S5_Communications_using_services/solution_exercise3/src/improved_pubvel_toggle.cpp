//This program is the solution for exercise 3a
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <solution_exercise3/Changerate.h>
#include <solution_exercise3/Changetwist.h>

// Global variables

// Flag to distinguish whether the turtle is
// moving forward or rotating
bool forward = true;
// Flag to indicate that toggle_forward service
// request has been taken into account
bool toggled = true;

// New input frequency
double newfrequency;
// Flag to indicate that change_rate service
// request has been taken into account 
bool ratechanged = false;

// Flag to indicate whether turtle is moving
// or stationary
bool moving = true;

// Flag to indicate that change_twist service
// has been taken into account
bool twistchanged = false;
geometry_msgs::Twist twist;

// Callback function for toggle_forward service
bool toggleForward(
	std_srvs::Empty::Request &req,
	std_srvs::Empty::Response &resp){
        
        if(moving){
                forward = !forward;
                twistchanged = false;
                toggled = true;
                ROS_INFO_STREAM("Now sending "<<(forward?
                "forward":"rotate")<< " commands.");
        }
        else{
                ROS_ERROR_STREAM("Failed to toggle as turtle is stationary");
        } 

	return true;
}

// Callback function for change_rate service
bool changeRate(
        solution_exercise3::Changerate::Request &req,
        solution_exercise3::Changerate::Response &resp){

        ROS_INFO_STREAM("Changing rate to "<<req.newrate);

        newfrequency = req.newrate;
        ratechanged = true;

        resp.ret = true;
        return true;
}

// Callback function for toggle_motion service
bool toggleMotion(
	std_srvs::Empty::Request &req,
	std_srvs::Empty::Response &resp){
        moving = !moving;
        ROS_INFO_STREAM("Now the turtle is "<<
                (moving? "moving":"stationary"));
	return true;
}

// Callback function for change_twist service
bool changeTwist(
        solution_exercise3::Changetwist::Request &req,
        solution_exercise3::Changetwist::Response &resp){
        
        if(moving){
                twist.linear.x = req.twist.linear.x;
                twist.linear.y = req.twist.linear.y;
                twist.linear.z = req.twist.linear.z;

                twist.angular.x = req.twist.angular.x;
                twist.angular.y = req.twist.angular.y;
                twist.angular.z = req.twist.angular.z;

                twistchanged = true;
                toggled = false;
                resp.ret = true;
                ROS_INFO_STREAM("Changing twist to ");
        }
        else{
                ROS_ERROR_STREAM("Failed to update twist as turtle is stationary");
                resp.ret = false;
        }

        

        return true;
}
 

int main(int argc, char **argv){
        // ROS init
        ros::init(argc,argv,"pubvel_toggle_rate");
	ros::NodeHandle nh;
        
	// Create the services that this node offers
        ros::ServiceServer server0 = 
		nh.advertiseService("toggle_forward",&toggleForward);
                
        ros::ServiceServer server1 =
                nh.advertiseService("change_rate",&changeRate);

        ros::ServiceServer server2 = 
		nh.advertiseService("toggle_motion",&toggleMotion);

        ros::ServiceServer server3 = 
		nh.advertiseService("change_twist",&changeTwist);
                
        // Create a publisher object
        ros::Publisher pub=nh.advertise<geometry_msgs::Twist>(
		"turtle1/cmd_vel",1000);
    
        ros::Rate rate(2);
	while(ros::ok()){
		geometry_msgs::Twist msg;
                if (moving){
                        
                        if(toggled){
                                msg.linear.x = forward? 1.0: 0.0;
                                msg.angular.z = forward? 0.0: 1.0;
                        }

                        if(twistchanged){
                                msg.linear.x = twist.linear.x;
                                msg.linear.y = twist.linear.y;
                                msg.linear.z = twist.linear.z;
                                msg.angular.x = twist.angular.x;
                                msg.angular.y = twist.angular.y;
                                msg.angular.z = twist.angular.z;
                        }
                        
                }
                else{
                        msg.linear.x = 0.0;
                        msg.angular.z = 0.0;
                }
                
		pub.publish(msg);
		ros::spinOnce();
                if(ratechanged) {
                    rate = ros::Rate(newfrequency);
                    ratechanged = false;
                }
		rate.sleep();
	}
}
