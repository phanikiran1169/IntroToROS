# Learnings

#### 1. ros::spinOnce() vs ros::spin():

ROS will only execute our callback function when we give it explicit permission to do so. There are actually two slightly different ways to accomplish this, using ros::spinOnce() or ros::spin().

1. ros::spinOnce() asks ROS to execute all of the pending callbacks from all of the node’s subscriptions, and then return control back to us.

2. ros::spin() asks ROS to wait for and execute callbacks until the node shuts down, i.e. it is roughly equivalent to this loop:

```
while(ros::ok()){
ros::spinOnce();
}
```
Test code

```
// This program subscribes to turtle1/pose and shows its
// messages on the screen.
#include <ros/ros.h>
#include <turtlesim/Pose.h>
#include <iomanip> // for std::setprecision and std::fixed

// Declare a global var to count the number of
// while() loop executions
int count = 0;

// A callback function.  Executed each time a new pose
// message arrives.
void poseMessageReceived(const turtlesim::Pose& msg) {
  ROS_INFO_STREAM(std::setprecision(2) << std::fixed
    << "position=(" <<  msg.x << "," << msg.y << ")"
    << " direction=" << msg.theta);
  
  // To display the nth while loop when callback
  // function is executed
  ROS_INFO_STREAM (count) ;
}

int main(int argc, char **argv) {
  // Initialize the ROS system and become a node.
  ros::init(argc, argv, "subscribe_to_pose");
  ros::NodeHandle nh;

  // Create a subscriber object.
  ros::Subscriber sub = nh.subscribe("turtle1/pose", 1000,
    &poseMessageReceived);

  ros::Rate rate(62);

  // Let ROS take over.
  
  // For understanding, the following while
  // loop is used instead of ros::spin()
  
  while (ros::ok()) {
    count++;
    ros::spinOnce();
    rate.sleep();
  }
  
}
```
