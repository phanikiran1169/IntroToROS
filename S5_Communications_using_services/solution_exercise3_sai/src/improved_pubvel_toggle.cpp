//this program toggles between rotation and translation
//commands,based on calls to a service.
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <solution_exercise3_sai/Changerate.h>
#include <solution_exercise3_sai/Changespeed.h>

bool forward = true;
double newfrequency;
bool ratechanged = false;
bool stop = true;
double newspeed;
bool speedchanged = false;

bool toggleForward(
	std_srvs::Empty::Request &req,
	std_srvs::Empty::Response &resp){
        forward = !forward;
        ROS_INFO_STREAM("Now sending "<<(forward?
                "forward":"rotate")<< " commands.");
	return true;
}

bool changeRate(
        solution_exercise3_sai::Changerate::Request &req,
        solution_exercise3_sai::Changerate::Response &resp){

        ROS_INFO_STREAM("Changing rate to "<<req.newrate);

        newfrequency = req.newrate;
        ratechanged = true;
        return true;
}

bool toggleStartStop(
        std_srvs::Empty::Request &req, 
        std_srvs::Empty::Response &resp)
{       
        ROS_INFO_STREAM((stop? "stopping":"starting") << "the turtle");
        stop = !stop;
        return true;
}

bool changeSpeed(
        solution_exercise3_sai::Changespeed::Request &req,
        solution_exercise3_sai::Changespeed::Response &resp)
{
        ROS_INFO_STREAM("Changing speed to "<<req.newvelocity);
        newspeed = req.newvelocity;
        speedchanged = true;
        return true;
}

int main(int argc, char **argv){
        ros::init(argc,argv,"pubvel_toggle_rate");
	ros::NodeHandle nh;
        
	ros::ServiceServer server = 
		nh.advertiseService("toggle_forward",&toggleForward);
                
        ros::ServiceServer server0 =
                nh.advertiseService("change_rate",&changeRate);

        ros::ServiceServer server1 =
                nh.advertiseService("toggle_startstop",&toggleStartStop);

        ros::ServiceServer server2 =
                nh.advertiseService("change_speed",&changeSpeed);
                
        ros::Publisher pub=nh.advertise<geometry_msgs::Twist>(
		"turtle1/cmd_vel",1000);
    
        ros::Rate rate(2);
	while(ros::ok()){
		geometry_msgs::Twist msg;
                // Use forward to check if toggle_foward is requested
                // velocities are multiplied by "stop" to include the
                // fucntionality of start / stop service.
                msg.linear.x = forward?stop*1.0:0.0;
                msg.angular.z=forward?0.0:stop*1.0;

                // If change_speed service is requested, overwrite the default
                // velocities with the new veolicities.
                if (speedchanged == true)
                {
                        msg.linear.x = forward?stop*newspeed:0.0;
                        msg.angular.z=forward?0.0:stop*newspeed;
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
