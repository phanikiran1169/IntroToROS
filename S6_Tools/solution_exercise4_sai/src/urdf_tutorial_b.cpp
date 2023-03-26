#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <solution_exercise4_sai/change_scale.h>

double deltaPan;
double deltaTilt;
double scale=0.5;
const double rad2deg = 180/M_PI;

ros::Publisher* joint_pub;

bool changeScale(
  solution_exercise4_sai::change_scale::Request &req, 
  solution_exercise4_sai::change_scale::Response &resp)
{
  ROS_INFO_STREAM("Changing the scale to:" << req.newscale);
  scale = req.newscale;
  return true;
}

void pubJointState(const geometry_msgs::Twist &msgIn)
{
  // message declarations
  static sensor_msgs::JointState joint_state;
  joint_state.name.resize(2);
  joint_state.position.resize(2);
  static double pan = 0.0;
  static double tilt = 0.0;

  // increments taken the keyboard input
  deltaPan =  msgIn.linear.x * rad2deg * scale;
  deltaTilt = msgIn.angular.z * rad2deg * scale;
        
  pan = pan + deltaPan;
  tilt = tilt + deltaTilt;
  
  //update joint_state
  joint_state.header.stamp = ros::Time::now();
  joint_state.name[0] ="pan_joint";
  joint_state.position[0] = pan;
  joint_state.name[1] ="tilt_joint";
  joint_state.position[1] = tilt;

  ROS_INFO_STREAM("pan_joint position:" << pan << "deg "
                  "& tilt_joint position: " << tilt << "deg");

  //publish the joint state 
  (*joint_pub).publish(joint_state);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "urdf_tutorial_b");
  ros::NodeHandle nh;

  // Create a server object for changing scale
	ros::ServiceServer server = nh.advertiseService("change_scale",&changeScale);

  //The node advertises the joint values of the pan-tilt
  joint_pub = new ros::Publisher(
                      nh.advertise<sensor_msgs::JointState>("joint_states", 1));

  // Create a subscriber object.
  ros::Subscriber sub = nh.subscribe("teleop_values", 1, &pubJointState);

  ros::spin();

  delete joint_pub;
}



