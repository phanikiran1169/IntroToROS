// This program publishes randomly-generated velocity
// messages for turtlesim.
#include <ros/ros.h>
#include <turtlesim/Pose.h>  // For turtlesim::Pose 
#include <geometry_msgs/Twist.h>  // For geometry_msgs::Twist
#include <stdlib.h> // For rand() and RAND_MAX
#include <iomanip> // for std::setprecision and std::fixed

/**
* A callback function to publish angular and linear velocities.
* The velocities are published in two patterns:
* 1. A random angular velocity command and a fixed linear velocity of 1.0 
*    when the turtle is placed in a safe zone defined by a square around
*    the center of the window.
* 2. A random angular velocity and a random linear velocity otherwise.
*
* Size of safe zone considered: 200 x 200 pixels
*/

void publishVelocities(const turtlesim::Pose& pose_msg, ros::Publisher& pub)
{
  ROS_INFO_STREAM(std::setprecision(2) << std::fixed
    << "current_position=(" <<  pose_msg.x << "," << pose_msg.y << ")");

  // Seed the random number generator.
  srand(time(0));

  // Loop at 2Hz until the node is shut down.
  ros::Rate rate(2);

  while(ros::ok())
    // Create and fill in the message.  The other four
    // fields, which are ignored by turtlesim, default to 0.
    geometry_msgs::Twist motion_msg;

  {
    if (pose_msg.x < 200 && pose_msg.y < 200)
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
    pub.publish(msg);

    // Send a message to rosout with the details.
    ROS_INFO_STREAM("Sending random velocity command:"
      << " linear=" << motion_msg.linear.x
      << " angular=" << motion_msg.angular.z);

    // Wait until it's time for another iteration.
    rate.sleep();
  }
  
}

int main(int argc, char **argv)
{
  // Initialize the ROS system and create publisher and subscriber node.
  ros::init(argc, argv, "publish_velocity_safe");
  ros::NodeHandle nh;

  // Create a publisher object.
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>(
    "turtle1/cmd_vel", 1000);

  // Create a subscriber object.
  ros::Subscriber sub = nh.subscribe("turtle1/pose", 1000, 
                                    boost::bind(publishVelocities, _1, pub));

  // Let ROS take over.
  ros::spin();
}
