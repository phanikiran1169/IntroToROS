#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <solution_exercise4_sai/change_scale.h>

double deltaShoulderPan=0.0;
double deltaShoulderPitch=0.0;
double deltaElbowRoll=0.0;
double deltaElbowPitch=0.0;
double deltaWristRoll=0.0;
double deltaWristPitch=0.0;
double deltaGripperRoll=0.0;
double deltaFinger1=0.0;
double deltaFinger2=0.0;
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
  joint_state.name.resize(9);
  joint_state.position.resize(9);
  static double shoulderPan = 0.0;
  static double shoulderPitch = 0.0;
  static double elbowRoll = 0.0;
  static double elbowPitch = 0.0;
  static double wristRoll = 0.0;
  static double wristPitch = 0.0;
  static double gripperRoll = 0.0;
  static double finger1 = 0.0;
  static double finger2 = 0.0;

  // increments taken the keyboard input
  deltaElbowPitch =  msgIn.linear.x * rad2deg * scale;
  deltaWristPitch = msgIn.angular.z * rad2deg * scale;
  
  shoulderPan = shoulderPan + deltaShoulderPan;
  shoulderPitch = shoulderPitch + deltaShoulderPitch;
  elbowRoll = elbowRoll + deltaElbowRoll;
  elbowPitch = elbowPitch + deltaElbowPitch;
  wristRoll = wristRoll + deltaWristRoll;
  wristPitch = wristPitch + deltaWristPitch;
  gripperRoll = gripperRoll + deltaGripperRoll;
  finger1 = finger1 + deltaFinger1;
  finger2 = finger2 + deltaFinger2;

  //update joint_state
  joint_state.header.stamp = ros::Time::now();
  joint_state.name[0] ="shoulder_pan_joint";
  joint_state.position[0] = shoulderPan;
  joint_state.name[1] ="shoulder_pitch_joint";
  joint_state.position[1] = shoulderPitch;
  joint_state.name[2] ="elbow_roll_joint";
  joint_state.position[2] = elbowRoll;
  joint_state.name[3] ="elbow_pitch_joint";
  joint_state.position[3] = elbowPitch;
  joint_state.name[4] ="wrist_roll_joint";
  joint_state.position[4] = wristRoll;
  joint_state.name[5] ="wrist_pitch_joint";
  joint_state.position[5] = wristPitch;
  joint_state.name[6] ="gripper_roll_joint";
  joint_state.position[6] = gripperRoll;
  joint_state.name[7] ="finger_joint1";
  joint_state.position[7] = finger1;
  joint_state.name[8] ="finger_joint2";
  joint_state.position[8] = finger2;

  ROS_INFO_STREAM("elbow_pitch_joint position:" << elbowPitch << "deg "
                  "& wrist_pitch_joint position: " << wristPitch << "deg");

  //publish the joint state 
  (*joint_pub).publish(joint_state);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "urdf_tutorial_arm");
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



