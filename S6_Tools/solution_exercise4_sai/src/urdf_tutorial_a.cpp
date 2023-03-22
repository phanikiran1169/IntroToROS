#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>


double deltaPan;
double deltaTilt;
double scale;

ros::Publisher* joint_pub;

void pubJointState(const geometry_msgs::Twist &msgIn)
{
  // message declarations
  static sensor_msgs::JointState joint_state;
  joint_state.name.resize(2);
  joint_state.position.resize(2);
  static double pan = 0.0;
  static double tilt = 0.0;
 
  scale =0.5;

  // increments taken the keyboard input
  deltaPan =  msgIn.linear.x * scale;
  deltaTilt = msgIn.angular.z * scale;
        
  pan = pan + deltaPan;
  tilt = tilt + deltaTilt;
  
  //update joint_state
  joint_state.header.stamp = ros::Time::now();
  joint_state.name[0] ="pan_joint";
  joint_state.position[0] = pan;
  joint_state.name[1] ="tilt_joint";
  joint_state.position[1] = tilt;

  //publish the joint state 
  (*joint_pub).publish(joint_state);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "urdf_tutorial_a");
  ros::NodeHandle nh;
  
  //The node advertises the joint values of the pan-tilt
  joint_pub = new ros::Publisher(
                      nh.advertise<sensor_msgs::JointState>("joint_states", 1));

  // Create a subscriber object.
  ros::Subscriber sub = nh.subscribe("teleop_values", 1, &pubJointState);

  ros::spin();

  delete joint_pub;
}



