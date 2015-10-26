#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <string.h>

#include <sensor_msgs/JointState.h>
#include <visualization_msgs/Marker.h>
#include <crawler_msgs/JointCmd.h>
#include <crawler_msgs/VisualHeading.h>


#define PI 3.14159265359

char ros_info_str [100];

// cmd_vel 
#define MMC_WhlLft_JointID  0
#define MMC_WhlRgt_JointID  1

#define MMC_CbwLft_JointID  2
#define MMC_CbwRgt_JointID  3

#define MMC_TrsBas_JointID  4

#define MMC_ArmYaw_JointID  5
#define MMC_ArmPic_JointID  6
#define MMC_ArmPmt_JointID  7

#define MMC_WrtRol_JointID  8
#define MMC_WrtPic_JointID  9

#define wheel_L MMC_WhlLft_JointID
#define wheel_R MMC_WhlRgt_JointID

double cmd_vel[2] = {0,0}; // command velocity

// VisualHeadingCallback
ros::Time visual_heading_last_time, visual_heading_current_time;

double visual_heading_current_Yaw = 0;
double visual_heading_last_Yaw = 0;

void VisualHeadingCallback(const crawler_msgs::VisualHeading& visual_heading_msg) {

  // time
  visual_heading_current_time = visual_heading_msg.header.stamp; //get new time stamp

  // data
  visual_heading_current_Yaw = visual_heading_msg.RPY_radian.z; // heading Yaw
  //visual_heading_current_Yaw = round(visual_heading_current_Yaw * 100) / 100;

  sprintf (ros_info_str, "Yaw = %f. \t Yaw_last = %f. \t time_now = %f, \t time_last = %f.", visual_heading_current_Yaw, visual_heading_last_Yaw, visual_heading_current_time.toSec(), visual_heading_last_time.toSec());
  ROS_INFO ("[CB]Heading: %s", ros_info_str);

}

// // VO_TEST // 
// geometry_msgs::PoseStamped visual_odometry;
// void VisualOdometryCallback(const geometry_msgs::PoseStamped& visual_odometry_msg) {
//   visual_odometry.pose.position = visual_odometry_msg.pose.position;
//   visual_odometry.pose.orientation = visual_odometry_msg.pose.orientation;

// }

// JointStateCallback
// ros::Time encoders_last_time encoders_current_time;
// double encoders[2] = {0,0};

// void JointStateCallback(const sensor_msgs::JointState& joint_states_msg) {

//   // // time
//   // encoders_last_time = encoders_current_time; //save the old time;
//   // encoders_current_time = joint_states_msg.header.stamp; //get new time stamp

//   // // data
//   // encoders[wheel_L] = (-1) * joint_states_msg.position[MMC_WhlLft_JointID]; // flip the left wheel encoder value sign.
//   // encoders[wheel_R] = joint_states_msg.position[MMC_WhlRgt_JointID];

//   // sprintf (ros_info_str, "L = %f \t R = %f. \t time_now = %f, \t time_last = %f.", encoders[wheel_L], encoders[wheel_R], encoders_current_time.toSec(), encoders_last_time.toSec());
//   // ROS_INFO ("Encoders: %s", ros_info_str);

// }

int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");

  ros::NodeHandle n;
  //ros::Subscriber joint_states_sub_ = n.subscribe("joint_states", 1, &JointStateCallback);
  //ros::Subscriber joint_cmd_sub_ = n.subscribe("/crawler/joint_cmd", 1, &JointCmdCallback);
  ros::Subscriber visual_heading_sub_ = n.subscribe("crawler/visual_heading", 1, &VisualHeadingCallback);
  // ros::Subscriber visual_odometry_sub_ = n.subscribe("visual_odometry/state", 1, &VisualOdometryCallback);
  //visual_odometry/state

  ros::Publisher cmd_vel_pub = n.advertise<crawler_msgs::JointCmd>("crawler/joint_cmd", 10);

  tf::TransformBroadcaster odom_broadcaster;

  double x = 0.0;
  double y = 0.0;
  double th = 0.0;

  double x = 0.0;
  double y = 0.0;
  double th = 0.0;

  double vx = 0.0;
  double vy = 0.0;
  double vth = 0.0;

  double crawler_pose_x = 0.0;
  double crawler_pose_y = 0.0;
  double crawler_pose_z = 0.0;
  double crawler_orient = 0.0;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(20); // 10Hz

  // check ros param
  double _crawler_init_pose_x, _crawler_init_pose_y, _crawler_init_pose_z, _crawler_init_pose_yaw;
  n.getParam("crawler_init_pose_x", _crawler_init_pose_x);
  n.getParam("crawler_init_pose_y", _crawler_init_pose_y);
  n.getParam("crawler_init_pose_z", _crawler_init_pose_z);
  n.getParam("crawler_init_pose_yaw", _crawler_init_pose_yaw);

  crawler_orient = _crawler_init_pose_yaw;
  crawler_pose_x = _crawler_init_pose_x;
  crawler_pose_y = _crawler_init_pose_y;
  crawler_pose_z = _crawler_init_pose_z;


  ROS_INFO ("Crawler Dead Reckoning Start...");

  while(n.ok()){

    ros::spinOnce();               // check for incoming messages
    current_time = ros::Time::now();


    
    double _use_visual_heading;
    n.getParam("use_visual_heading", _use_visual_heading);

    double dt = (current_time - last_time).toSec();

    ROS_INFO (" ");

    r.sleep();
  }
}
