//this program toggles between rotation and translation
//commands,based on calls to a service.
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <demodoc/Changerate.h>

bool forward = true;
double newfrequency;
bool ratechanged = false;


/*!
    * \brief toggleForward is a sevice that allows you to toggle between translational and rotational velocities to be published.
    *
    * Its arguments are the following:
    * \param req This is the request argument. No request argument is needed so it is of type empty std_srvs::Empty
    * \param resp This is the response argument. No request argument is needed so it is of type empty std_srvs::Empty
    */
bool toggleForward(
	std_srvs::Empty::Request &req,
	std_srvs::Empty::Response &resp){
        forward = !forward;
        ROS_INFO_STREAM("Now sending "<<(forward?
                "forward":"rotate")<< " commands.");
	return true;
}

/*!
    * \brief changeRate is a sevice that allows you to change the rate at which the node is publishing the commanded velocities to the turtle.
    *
    * Its arguments are the following:
    * \param req This is the request argument. 
    * \param resp This is the response argument. 
    */
bool changeRate(
        demodoc::Changerate::Request &req,
        demodoc::Changerate::Response &resp){

        ROS_INFO_STREAM("Changing velocity to "<<req.newrate);

        newfrequency = req.newrate;
        ratechanged = true;
        return true;
}

/*!
    * \brief The main function creates the node that:
    *  a) publishes commanded velocities of type <geometry_msgs::Twist> to the topic "turtle1/cmd_vel". 
    *  b) offers a service to change the publishing rate
    *  c) offers a service to toggle between translational and rotational velocity.
    *
    * No parameters are used.
    */
int main(int argc, char **argv){
        ros::init(argc,argv,"pubvel_toggle");
	ros::NodeHandle nh;
        
	ros::ServiceServer server = 
		nh.advertiseService("toggle_forward",&toggleForward);
                
        ros::ServiceServer server0 =
                nh.advertiseService("change_rate",&changeRate);
                
        ros::Publisher pub=nh.advertise<geometry_msgs::Twist>(
		"turtle1/cmd_vel",1000);
    
        ros::Rate rate(2);
	while(ros::ok()){
		geometry_msgs::Twist msg;
                msg.linear.x = forward?1.0:0.0;
                msg.angular.z=forward?0.0:1.0;
		pub.publish(msg);
		ros::spinOnce();
                if(ratechanged) {
                    rate = ros::Rate(newfrequency);
                    ratechanged = false;
                }
		rate.sleep();
	}
}
