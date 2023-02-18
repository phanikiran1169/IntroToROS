// This program subscribes to turtle1/cmd_vel and
// republishes on turtle1/cmd_vel_reversed,
// with the signs inverted.
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

ros::Publisher *pubPtr;

void commandVelocityReceived(
  const geometry_msgs::Twist& msgIn
) {
  geometry_msgs::Twist msgOut;
  msgOut.linear.x = -msgIn.linear.x;
  msgOut.angular.z = -msgIn.angular.z;
  pubPtr->publish(msgOut);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "reverse_velocity");
  ros::NodeHandle nh;

  pubPtr = new ros::Publisher(
    nh.advertise<geometry_msgs::Twist>(
      "turtle1/cmd_vel_reversed",
      1000));

  ros::Subscriber sub = nh.subscribe(
    "turtle1/cmd_vel", 1000,
    &commandVelocityReceived);

  ros::spin();

  delete pubPtr;
}

