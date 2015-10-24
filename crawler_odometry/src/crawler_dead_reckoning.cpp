#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <string.h>

#include <sensor_msgs/JointState.h>
#include <crawler_msgs/JointCmd.h>

char ros_info_str [100];


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

// x = cmd_vel = [0.5,0.75,1]
// y = wheel_speed = [0.0059153164,0.0105673675,0.0152072592]
// pk =     0.01858;
// pb =   -0.003375;
// y = pk*x + pb;

#define wheel_L MMC_WhlLft_JointID
#define wheel_R MMC_WhlRgt_JointID

double cmd_vel[2] = {0,0};
double wheel_speed[2] = {0,0};
static const double wheel_speed_pk = 0.01858;
static const double wheel_speed_pb = -0.003375;

static const double wheel_width = 0.27;

ros::Time cmd_vel_current_time, cmd_vel_last_time; 

void JointCmdCallback(const crawler_msgs::JointCmd& joint_cmd_msg) {

  // time
  cmd_vel_last_time = cmd_vel_current_time; //save the old time;
  cmd_vel_current_time = joint_cmd_msg.header.stamp; //get new time stamp

  // data
  cmd_vel[wheel_L] = (-1) * joint_cmd_msg.jointCmdVel[MMC_WhlLft_JointID]; // flip the left wheel encoder value sign.
  cmd_vel[wheel_R] = joint_cmd_msg.jointCmdVel[MMC_WhlRgt_JointID];

  //sprintf (ros_info_str, "L = %f \t R = %f. \t time_now = %f, \t time_last = %f.", cmd_vel[wheel_L], cmd_vel[wheel_R], encoders_current_time.toSec(), encoders_last_time.toSec());
  //ROS_INFO ("Encoders: %s", ros_info_str);

}

void JointStateCallback(const sensor_msgs::JointState& joint_states_msg) {

  // // time
  // encoders_last_time = encoders_current_time; //save the old time;
  // encoders_current_time = joint_states_msg.header.stamp; //get new time stamp

  // // data
  // encoders[wheel_L] = (-1) * joint_states_msg.position[MMC_WhlLft_JointID]; // flip the left wheel encoder value sign.
  // encoders[wheel_R] = joint_states_msg.position[MMC_WhlRgt_JointID];

  // sprintf (ros_info_str, "L = %f \t R = %f. \t time_now = %f, \t time_last = %f.", encoders[wheel_L], encoders[wheel_R], encoders_current_time.toSec(), encoders_last_time.toSec());
  // ROS_INFO ("Encoders: %s", ros_info_str);

}

int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");

  ros::NodeHandle n;
  ros::Subscriber joint_states_sub_ = n.subscribe("joint_states", 1, &JointStateCallback);
  ros::Subscriber joint_cmd_sub_ = n.subscribe("/crawler/joint_cmd", 1, &JointCmdCallback);

  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  tf::TransformBroadcaster odom_broadcaster;

  double x = 0.0;
  double y = 0.0;
  double th = 0.0;

  double vx = 0.0;
  double vy = 0.0;
  double vth = 0.0;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(1.0);
  while(n.ok()){

    ros::spinOnce();               // check for incoming messages
    current_time = ros::Time::now();

    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();

    wheel_speed[wheel_L] = wheel_speed_pk * cmd_vel[wheel_L] + wheel_speed_pb;
    wheel_speed[wheel_R] = wheel_speed_pk * cmd_vel[wheel_R] + wheel_speed_pb;

    sprintf (ros_info_str, "L = %f \t R = %f. \t time_now = %f, \t time_last = %f.", wheel_speed[wheel_L], wheel_speed[wheel_R], cmd_vel_current_time.toSec(), cmd_vel_last_time.toSec());
    ROS_INFO ("wheel_speed: %s", ros_info_str);


    // real speed is the midsegment of a trapezoid form by wheel_speed_L, wheel_speed_R, and wheel_width.
    vx = (wheel_speed[wheel_L] + wheel_speed[wheel_R])/2;

    double delta_x = vx * dt;
    double delta_y = vy * dt;
    double delta_th = vth * dt;

    x += delta_x;
    y += delta_y;
    th += delta_th;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
    r.sleep();
  }
}
