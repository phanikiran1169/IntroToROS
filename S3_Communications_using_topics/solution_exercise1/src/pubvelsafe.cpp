// This program publishes randomly-generated velocity
// messages for turtlesim.
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>  // For geometry_msgs::Twist
#include <std_msgs/Float32.h>
#include <turtlesim/Pose.h>
#include <stdlib.h> // For rand() and RAND_MAX

// Global variable to store the pose of turtlesim
turtlesim::Pose pose;

// Centre of turtlesim window
const float centreX = 5.5;
const float centreY = 5.5;

// Safety margin (Defined by half length 
// of the square safety window) 
const float margin = 1.0;

// function to find if given point
// lies inside a given square or not.
bool liesInside(float x, float y)
{
    float x1 = centreX - margin;
    float y1 = centreY - margin;

    float x2 = centreX + margin;
    float y2 = centreY + margin;

    if (x > x1 and x < x2 and y > y1 and y < y2)
        return true;
 
    return false;
}

// A callback function.  Executed each time a new pose
// message arrives.
void poseMessageReceived(const turtlesim::Pose& msg) {
  ROS_INFO_STREAM(std::setprecision(2) << std::fixed
    << "position=(" <<  msg.x << "," << msg.y << ")"
    << " direction=" << msg.theta);

    pose = msg;
}

int main(int argc, char **argv) {
  
  
  // Initialize the ROS system and become a node.
  ros::init(argc, argv, "publish_velocity");
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
  ros::Rate rate(2);

  while(ros::ok()) {
    // Create and fill in the message.  The other four
    // fields, which are ignored by turtlesim, default to 0.
    geometry_msgs::Twist twist;
    
    if(liesInside(pose.x, pose.y)) {
      ROS_INFO_STREAM("Turtle is inside safe zone");
      twist.linear.x = 1.0;
      twist.angular.z = 2*double(rand())/double(RAND_MAX) - 1;
    }

    else {
      ROS_INFO_STREAM("Turtle is outside safe zone");
      twist.linear.x = double(rand())/double(RAND_MAX);
      twist.angular.z = 2*double(rand())/double(RAND_MAX) - 1;
    }

    // Publish the message.
    pub.publish(twist);

    // Send a message to rosout with the details.
    ROS_INFO_STREAM("Sending random velocity command:"
      << " linear=" << twist.linear.x
      << " angular=" << twist.angular.z);

    ros::spinOnce();
    
    // Wait until it's time for another iteration.
    rate.sleep();
  }
}
