//This program spawns a new turtlesim turtle by calling
// the appropriate service.
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>

//The srv class for the service.
#include <turtlesim/Spawn.h>
#include <solution_exercise3_sai/Changespeed.h>

ros::Publisher *pubPtr;

void newPubvel(const geometry_msgs::Twist &msgIn)
{
    geometry_msgs::Twist msgOut;
    msgOut.linear.x = msgIn.linear.x;
    msgOut.angular.z = msgIn.angular.z;
    (*pubPtr).publish(msgOut);
}

int main(int argc, char **argv){

    ros::init(argc, argv, "spawn_turtle_plus");
    ros::NodeHandle nh;

// Create a client object for the spawn service. This needs to know the data type 
// of the service and its name.
    ros::ServiceClient spawnClient
		= nh.serviceClient<turtlesim::Spawn>("spawn");

    //Create the request and response objects.
    turtlesim::Spawn::Request req;
    turtlesim::Spawn::Response resp;

    req.x = 2;
    req.y = 3;
    req.theta = M_PI/2;
    req.name = "MyTurtle";

    ros::service::waitForService("spawn", ros::Duration(5));
    bool success = spawnClient.call(req,resp);

    if(success){
	ROS_INFO_STREAM("Spawned a turtle named "
			<< resp.name);
    }else{
	ROS_ERROR_STREAM("Failed to spawn.");
    }

// Create a publisher object.
    pubPtr = new nh.advertise<geometry_msgs::Twist>("MyTurtle/cmd_vel", 1000);

// Create a subscriber object.
    ros::Subscriber sub = nh.subscribe("turtle1/pose", 1000, &newPubvel);

// Create a client object for toggle forward service.
    ros::ServiceClient toggleforwardClient 
        = nh.serviceClient<std_srvs::Empty>("toggle_forward");

    // Create the request and response objects for toggle forward service.
    std_srvs::Empty::Request req1;
    std_srvs::Empty::Response resp1;

    ros::service::waitForService("toggle_forward", ros::Duration(5));
    bool success = toggleforwardClient.call(req1,resp1);

    if(success)
    {
	    ROS_INFO_STREAM("Toggle forward triggered. Turtles switching motion");
    }
    else
    {
	    ROS_ERROR_STREAM("Failed to trigger toggle forward.");
    }

// Create a client object for change speed service.
    ros::ServiceClient changeSpeedClient 
        = nh.serviceClient<solution_exercise3_sai::Changespeed>("change_speed");

    // Create the request and response objects for toggle forward service.
    solution_exercise3_sai::Changespeed::Request req2;
    solution_exercise3_sai::Changespeed::Response resp2;

    req2.newvelocity = 10

    ros::service::waitForService("change_speed", ros::Duration(5));
    bool success = changeSpeedClient.call(req2,resp2);

    if(success)
    {
	    ROS_INFO_STREAM("Set the initial velocity to " <<req2.newvelocity);
    }
    else
    {
	    ROS_ERROR_STREAM("Failed to set initial velocity.");
    }

    ros::spin();

    delete pubPtr;
}
