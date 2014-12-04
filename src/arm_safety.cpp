#include <carl_safety/arm_safety.h>

using namespace std;

arm_safety::arm_safety()
{
  // a private handle for this ROS node (allows retrieval of relative parameters)
  ros::NodeHandle private_nh("~");
  private_nh.param<bool>("enable_audible_warnings", enable_audible_warnings, false);

  // create the ROS topics
  joint_sub = node.subscribe<sensor_msgs::JointState>("/jaco_arm/joint_states", 10, &arm_safety::joints_cback, this);
}



void arm_safety::joints_cback(const sensor_msgs::JointState::ConstPtr& joints)
{
  bool shouldSpeak = false;

  if(abs(joints->effort[0])>LARGE_ACTUATOR_THRESHOLD){
    ROS_ERROR("Torque on jaco_joint_1 outside threshold (+/-%f Nm) with value %f", LARGE_ACTUATOR_THRESHOLD, joints->effort[0]);
    shouldSpeak = true;
  }
  if(abs(joints->effort[1])>LARGE_ACTUATOR_THRESHOLD){
    ROS_ERROR("Torque on jaco_joint_2 outside threshold (+/-%f Nm) with value %f", LARGE_ACTUATOR_THRESHOLD, joints->effort[1]);
    shouldSpeak = true;
  }
  if(abs(joints->effort[2])>LARGE_ACTUATOR_THRESHOLD){
    ROS_ERROR("Torque on jaco_joint_3 outside threshold (+/-%f Nm) with value %f", LARGE_ACTUATOR_THRESHOLD, joints->effort[2]);
    shouldSpeak = true;
  }
  if(abs(joints->effort[3])>SMALL_ACTUATOR_THRESHOLD){
    ROS_ERROR("Torque on jaco_joint_4 outside threshold (+/-%f Nm) with value %f", SMALL_ACTUATOR_THRESHOLD, joints->effort[3]);
    shouldSpeak = true;
  }
  if(abs(joints->effort[4])>SMALL_ACTUATOR_THRESHOLD){
    ROS_ERROR("Torque on jaco_joint_5 outside threshold (+/-%f Nm) with value %f", SMALL_ACTUATOR_THRESHOLD, joints->effort[4]);
    shouldSpeak = true;
  }
  if(abs(joints->effort[5])>SMALL_ACTUATOR_THRESHOLD){
    ROS_ERROR("Torque on jaco_joint_6 outside threshold (+/-%f Nm) with value %f", SMALL_ACTUATOR_THRESHOLD, joints->effort[5]);
    shouldSpeak = true;
  }
  if(abs(joints->effort[6])>FINGER_ACTUATOR_THRESHOLD){
    ROS_ERROR("Torque on jaco_joint_6 outside threshold (+/-%f Nm) with value %f", FINGER_ACTUATOR_THRESHOLD, joints->effort[6]);
    shouldSpeak = true;
  }
  if(abs(joints->effort[7])>FINGER_ACTUATOR_THRESHOLD){
    ROS_ERROR("Torque on jaco_joint_6 outside threshold (+/-%f Nm) with value %f", FINGER_ACTUATOR_THRESHOLD, joints->effort[7]);
    shouldSpeak = true;
  }
  if(abs(joints->effort[8])>FINGER_ACTUATOR_THRESHOLD){
    ROS_ERROR("Torque on jaco_joint_6 outside threshold (+/-%f Nm) with value %f", FINGER_ACTUATOR_THRESHOLD, joints->effort[8]);
    shouldSpeak = true;
  }

  if(shouldSpeak && enable_audible_warnings)
    system("espeak \"ouch\"");
}

int main(int argc, char **argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "arm_safety");

  arm_safety a;
  ros::Rate loop_rate(60);
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  //fh.close();

  return EXIT_SUCCESS;
}
