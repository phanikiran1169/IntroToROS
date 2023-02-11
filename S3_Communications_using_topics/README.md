# Learnings

#### 1. ros::spinOnce() vs ros::spin()

ROS will only execute our callback function when we give it explicit permission to do so. There are actually two slightly different ways to accomplish this, using ros::spinOnce() or ros::spin().

1. ros::spinOnce() asks ROS to execute all of the pending callbacks from all of the node’s subscriptions, and then return control back to us.

2. ros::spin() asks ROS to wait for and execute callbacks until the node shuts down, i.e. it is roughly equivalent to this loop:

```
while(ros::ok()){
ros::spinOnce();
}
```
Test code: subpose.cpp

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
  
  // Vary the callback frequency and
  // see how the count variable changes
  
  // Here 62 is taken because the data is published
  // at the avg rate of 62 Hz on the topic
  // turtle1/pose
  // NOTE that the above frequency is not exact
  // and can vary depending on the sampling window
  
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
Frequency at which data is being published on turtle1/pose topic

![](https://user-images.githubusercontent.com/17789814/218255483-bfb24121-7848-400a-9618-a74e0a5c1fb4.png)



**Test1**: Used ros::spinOnce() without ros::Rate rate(X) command <br>
**Results**: Whileloop was excecuted 14536461 times before the callback function was called for the first time, followed by X(15327062 - 14536461) times of while loop execution before next call for callback function execution. This means the ros::spinOnce() was executed 14536460 times but there was no new message published on the topic for callback function execution. Similar logic can be extended for next steps.

![](https://user-images.githubusercontent.com/17789814/218255394-bf3640f8-2bc5-4d61-bcb3-ecc51bf332c0.png)

**Test2**: Used ros::spinOnce() with ros::Rate rate(62) command <br>
**Results**: Whileloop was excecuted 20 times before the callback function was called for the first time, followed by ~single while loop execution before next call for callback function execution. This means the ros::spinOnce() was executed 20 times but there was no new message published on the topic for callback function execution. Since we matched the ros::spinOnce() execution rate with publishing message rate, we observe that for approximately every ros::spinOnce() execution, the callback function is called.

![](https://user-images.githubusercontent.com/17789814/218255420-48816042-3cd0-4ac0-b200-10649c12c891.png)

**ProTip** 💡 : Unless there is a need to control the subscribing frequency, use ros::spin() to avoid hassle.
