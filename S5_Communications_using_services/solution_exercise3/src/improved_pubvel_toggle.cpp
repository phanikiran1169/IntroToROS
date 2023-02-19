//this program toggles between rotation and translation
//commands,based on calls to a service.
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <solution_exercise3/Changerate.h>
#include <solution_exercise3/Changetwist.h>

bool forward = true;
double newfrequency;
bool ratechanged = false;
bool move = true;

bool toggleForward(
	std_srvs::Empty::Request &req,
	std_srvs::Empty::Response &resp){
        
        if(move){
                forward = !forward;
                ROS_INFO_STREAM("Now sending "<<(forward?
                "forward":"rotate")<< " commands.");
        }
        else{
                ROS_ERROR_STREAM("Failed to toggle as turtle is stationary");
        } 

	return true;
}

bool changeRate(
        solution_exercise3::Changerate::Request &req,
        solution_exercise3::Changerate::Response &resp){

        ROS_INFO_STREAM("Changing rate to "<<req.newrate);

        newfrequency = req.newrate;
        ratechanged = true;

        resp.ret = true;
        return true;
}

bool toggleMotion(
	std_srvs::Empty::Request &req,
	std_srvs::Empty::Response &resp){
        move = !move;
        ROS_INFO_STREAM("Now the turtle is "<<
                (move? "moving":"stationary"));
	return true;
}

bool changeTwist(
        solution_exercise3::Changetwist::Request &req,
        solution_exercise3::Changetwist::Response &resp){


        return true;
}
 

int main(int argc, char **argv){
        ros::init(argc,argv,"pubvel_toggle_rate");
	ros::NodeHandle nh;
        
	ros::ServiceServer server0 = 
		nh.advertiseService("toggle_forward",&toggleForward);
                
        ros::ServiceServer server1 =
                nh.advertiseService("change_rate",&changeRate);

        ros::ServiceServer server2 = 
		nh.advertiseService("toggle_motion",&toggleMotion);        
                
        ros::Publisher pub=nh.advertise<geometry_msgs::Twist>(
		"turtle1/cmd_vel",1000);
    
        ros::Rate rate(2);
	while(ros::ok()){
		geometry_msgs::Twist msg;
                if (move){
                        msg.linear.x = forward?1.0:0.0;
                        msg.angular.z=forward?0.0:1.0;
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
