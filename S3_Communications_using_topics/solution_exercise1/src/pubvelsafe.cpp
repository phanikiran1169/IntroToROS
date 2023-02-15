// This program publishes randomly-generated velocity
// messages for turtlesim.
#include <ros/ros.h>
#include <turtlesim/Pose.h>  // For turtlesim::Pose 
#include <geometry_msgs/Twist.h>  // For geometry_msgs::Twist
#include <stdlib.h> // For rand() and RAND_MAX
#include <iomanip> // for std::setprecision and std::fixed

// Declare global postion variables
float position_x;
float position_y;

// Declare global variable for checking callback function exceution
bool callback_trigger = false;

// A callback function.  Executed each time a new pose
// message arrives.
void poseMessageReceived(const turtlesim::Pose& msg) 
{
  ROS_INFO_STREAM(std::setprecision(2) << std::fixed
    << "current_position=(" <<  msg.x << "," << msg.y << ")");
  position_x = msg.x;
  position_y = msg.y;
  callback_trigger = true;
}

int main(int argc, char **argv)
{
  // Initialize the ROS system and become a node.
  ros::init(argc, argv, "publish_velocity_safe");
  ros::NodeHandle nh;

  // Create a publisher object.
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>(
    "turtle1/cmd_vel", 1000);

  // Create a subscriber object.
  ros::Subscriber sub = nh.subscribe("turtle1/pose", 1000,
    &poseMessageReceived);

  // Seed the random number generator.
  srand(time(0));

  // Loop at 2Hz until the node is shut down.
  ros::Rate rate(10);

  while(ros::ok())
  {
    // Create and fill in the message.  The other four
    // fields, which are ignored by turtlesim, default to 0.
    geometry_msgs::Twist motion_msg;

    // Let ROS take over
    while(true)
    {
      ros::spinOnce();

      if (callback_trigger == true)
      {
        break;
      }
    }

    if (position_x < 200 && position_y < 200)
    {
      ROS_INFO_STREAM("Turtle is inside safe zone");
      motion_msg.linear.x = 1.0;
      motion_msg.angular.z = 2*double(rand())/double(RAND_MAX) - 1;
    }
    else
    {
      ROS_INFO_STREAM("Turtle is outside safe zone");
      motion_msg.linear.x = double(rand())/double(RAND_MAX);
      motion_msg.angular.z = 2*double(rand())/double(RAND_MAX) - 1;
    }

    // Publish the message.
    pub.publish(motion_msg);

    // Send a message to rosout with the details.
    ROS_INFO_STREAM("Sending random velocity command:"
      << " linear=" << motion_msg.linear.x
      << " angular=" << motion_msg.angular.z);

    callback_trigger = false;

    // Wait until it's time for another iteration.
    rate.sleep();
  }
}