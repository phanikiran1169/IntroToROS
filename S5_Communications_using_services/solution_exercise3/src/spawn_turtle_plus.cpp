//This program is the solution for exercise 3b
#include <ros/ros.h>
//The srv class for the service.
#include <turtlesim/Spawn.h>
#include <std_srvs/Empty.h>
#include <solution_exercise3/Changetwist.h>
#include <geometry_msgs/Twist.h>

ros::Publisher *pubPtr;

void commandVelocityReceived(const geometry_msgs::Twist& msgIn){
    geometry_msgs::Twist msgOut;
    msgOut.linear.x = msgIn.linear.x;
    msgOut.angular.z = msgIn.angular.z;
    pubPtr->publish(msgOut);
}

int main(int argc, char **argv){

    ros::init(argc, argv, "spawn_turtle");
    ros::NodeHandle nh;

//Create a client object for the spawn service. This
//needs to know the data type of the service and its name.
    ros::ServiceClient spawnClient
		= nh.serviceClient<turtlesim::Spawn>("spawn");

//Create the request and response objects.
    turtlesim::Spawn::Request req0;
    turtlesim::Spawn::Response resp0;

    req0.x = 2;
    req0.y = 3;
    req0.theta = M_PI/2;
    req0.name = "Leo";

    ros::service::waitForService("spawn", ros::Duration(5));
    bool success = spawnClient.call(req0,resp0);

    if(success){
	ROS_INFO_STREAM("Spawned a turtle named "
			<< resp0.name);
    }else{
	ROS_ERROR_STREAM("Failed to spawn.");
    }

//Create the request and response objects.
    std_srvs::Empty::Request req1;
    std_srvs::Empty::Response resp1;    
    
    ros::ServiceClient toggleForwardClient
		= nh.serviceClient<std_srvs::Empty>("toggle_forward");

    ros::service::waitForService("toggle_forward", ros::Duration(5));
    success = toggleForwardClient.call(req1,resp1);

    if(success){
	ROS_INFO_STREAM("Forward motion toggled. Now turtle starts to rotate");
    }else{
	ROS_ERROR_STREAM("Failed to toggle forward motion.");
    }

    pubPtr = new ros::Publisher(nh.advertise<geometry_msgs::Twist>("Leo/cmd_vel", 1000));

    ros::Subscriber sub = nh.subscribe("turtle1/cmd_vel", 1000, &commandVelocityReceived);

    ros::spin();

    delete pubPtr;

}
