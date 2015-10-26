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
double wheel_speed[2] = {0,0}; // wheel linear speed (in meter)

// Calculate command vel to real linear speed.
// x = cmd_vel = [0.5,0.75,1]
// y = wheel_speed = [0.0059153164,0.0105673675,0.0152072592]
// pk =     0.01858;
// pb =   -0.003375;
// y = pk*x + pb;

static const double wheel_speed_pk = 0.01858;
static const double wheel_speed_pb = -0.003375;
static const double wheel_width = 0.27;

// JointCmdCallback
ros::Time cmd_vel_current_time, cmd_vel_last_time; 

void JointCmdCallback(const crawler_msgs::JointCmd& joint_cmd_msg) {

  // time
  cmd_vel_current_time = joint_cmd_msg.header.stamp; //get new time stamp

  // data
  cmd_vel[wheel_L] =  joint_cmd_msg.jointCmdVel[MMC_WhlLft_JointID]; // flip the left wheel encoder value sign.
  cmd_vel[wheel_R] = (-1) * joint_cmd_msg.jointCmdVel[MMC_WhlRgt_JointID];

  sprintf (ros_info_str, "L = %f \t R = %f. \t time_now = %f, \t time_last = %f.", cmd_vel[wheel_L], cmd_vel[wheel_R], cmd_vel_current_time.toSec(), cmd_vel_last_time.toSec());
  ROS_INFO ("[CB]Encoders: %s", ros_info_str);

}

// VisualHeadingCallback
ros::Time visual_heading_last_time, visual_heading_current_time;

double visual_heading_current_Yaw = 0;
double visual_heading_last_Yaw = 0;

void VisualHeadingCallback(const crawler_msgs::VisualHeading& visual_heading_msg) {

  // time
  visual_heading_current_time = visual_heading_msg.header.stamp; //get new time stamp

  // data
  visual_heading_current_Yaw = visual_heading_msg.RPY_radian.z; // heading Yaw
  visual_heading_current_Yaw = round(visual_heading_current_Yaw * 100) / 100;

  sprintf (ros_info_str, "Yaw = %f. \t time_now = %f, \t time_last = %f.", visual_heading_current_Yaw, visual_heading_current_time.toSec(), visual_heading_last_time.toSec());
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
  ros::Subscriber joint_cmd_sub_ = n.subscribe("/crawler/joint_cmd", 1, &JointCmdCallback);
  ros::Subscriber visual_heading_sub_ = n.subscribe("crawler/visual_heading", 1, &VisualHeadingCallback);
  // ros::Subscriber visual_odometry_sub_ = n.subscribe("visual_odometry/state", 1, &VisualOdometryCallback);
  //visual_odometry/state


  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  ros::Publisher pose_pub = n.advertise<geometry_msgs::TwistStamped>("crawler/pose", 50);
  ros::Publisher crawler_vis_pub = n.advertise<visualization_msgs::Marker>( "pose_rviz_marker", 0 );
  ros::Publisher wingbay_vis_pub = n.advertise<visualization_msgs::Marker>( "wingbayrviz_marker", 0 );

  tf::TransformBroadcaster odom_broadcaster;

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
    // if (n.getParam("use_visual_heading", _use_visual_heading))
    // {
    //   ROS_INFO("Got _use_visual_heading: %f", _use_visual_heading);
    // }
    // else
    // {
    //   ROS_ERROR("Failed to get param 'use_visual_heading'");
    // }

    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();

    
    

    if (fabs(cmd_vel[wheel_L]) <= fabs(wheel_speed_pb / wheel_speed_pk)) {
      wheel_speed[wheel_L] = 0.0;
      ROS_INFO ("L01");
    } else {
      double sign_wheel_L = cmd_vel[wheel_L] / fabs(cmd_vel[wheel_L]);
      wheel_speed[wheel_L] = (wheel_speed_pk * fabs(cmd_vel[wheel_L]) + wheel_speed_pb) * sign_wheel_L;
      ROS_INFO ("L02, sign %f",sign_wheel_L);
    }

    if (fabs(cmd_vel[wheel_R]) <= fabs(wheel_speed_pb / wheel_speed_pk)) {
      wheel_speed[wheel_R] = 0.0;
      ROS_INFO ("R01");
    } else {
      double sign_wheel_R = cmd_vel[wheel_R] / fabs(cmd_vel[wheel_R]);

      wheel_speed[wheel_R] = (wheel_speed_pk * fabs(cmd_vel[wheel_R]) + wheel_speed_pb) * sign_wheel_R;
      ROS_INFO ("R02, sign %f",sign_wheel_R);
    }    

    // setprecision set to 0.01
    //wheel_speed[wheel_L] = round(wheel_speed[wheel_L] * 100) / 100;
    //wheel_speed[wheel_R] = round(wheel_speed[wheel_R] * 100) / 100;

    sprintf (ros_info_str, "cmd_vel: L= %f,\tR= %f. \t w_spt: L= %f,\tR= %f.", cmd_vel[wheel_L], cmd_vel[wheel_R], wheel_speed[wheel_L], wheel_speed[wheel_R]);
    ROS_INFO ("[ML]%s", ros_info_str);


    // real speed is the midsegment of a trapezoid form by wheel_speed_L, wheel_speed_R, and wheel_width.
    double v_linear = ((wheel_speed[wheel_L] + wheel_speed[wheel_R])/2);

    // Use visual heading only if that node is open
    // if ((visual_heading_current_time !=0) && (visual_heading_last_time !=0)) {

    // }

    // calculate theta.
    double delta_th = 0;
    if (_use_visual_heading == 1.0) { 
      // using visual heading.
      delta_th = visual_heading_current_Yaw - visual_heading_last_Yaw;
      th = visual_heading_current_Yaw;
      vth = delta_th / dt;    
    } else if (_use_visual_heading == 0.0) { 
      // use wheel speed to calculate heading angle.
      // CCW is possitive heading angle, then wheel speed R>L
      double wheel_diff = wheel_speed[wheel_R]-wheel_speed[wheel_L];
      //vth = atan(fabs(wheel_diff) / wheel_width) * (wheel_diff/fabs(wheel_diff);  // atan angle * sign
      //vth = atan2( wheel_diff, wheel_width );

      // when angle is small enough tan(t) ~= t
      // http://www.wolframalpha.com/input/?i=plot+y%3D+tan%28x%29+and+y%3Dx+%28x+from+-1+to+1%29
      vth = wheel_diff / wheel_width;
      delta_th = vth * dt;
      th += delta_th;

      sprintf (ros_info_str, "wheel_diff = %f, \t wheel_diff / wheel_width = %f.", wheel_diff, (wheel_diff / wheel_width));
      ROS_INFO ("[ML]%s", ros_info_str);


    } else {
      ROS_INFO ("[ML]rosparam <use_visual_heading> setup incorrectly! (should be 0.0 or 1.0)");
    }
    
    vx = v_linear * sin (vth)* (-1);
    vy = v_linear * cos (vth);

    double delta_y = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_x = (vx * sin(th) + vy * cos(th)) * dt;

    x += delta_x;
    y += delta_y;    

    //x += vx * dt;
    //y += vy * dt;

    sprintf (ros_info_str, "v_linear = %f.\tvx= %f,\tvy=%f.\tx=%f,\ty=%f.\tdt=%f.", v_linear, vx, vy, x, y, dt);
    ROS_INFO ("[ML]%s", ros_info_str);

    sprintf (ros_info_str, "vth = %f.\tdelta_th= %f,\tth_current=%f.\tth_current_last=%f.", vth, delta_th, th, visual_heading_last_Yaw);
    ROS_INFO ("[ML]%s", ros_info_str);

    //double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    //double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    //double delta_th = vth * dt;

    //dt_th = (visual_heading_current_time - visual_heading_last_time).toSec();
    
    // v theta 
    //vth = delta_th / dt_th;


    //x += delta_x;
    //y += delta_y;
    //th += delta_th;

    // publish pose
    crawler_orient = std::fmod( crawler_orient + vth * dt, 2*PI);
    // crawler_pose_x += std::sin(crawler_orient + PI/2.0) * v_linear * dt;
    crawler_pose_x += std::sin(crawler_orient + PI/2.0) * v_linear * dt;
    // crawler_pose_y += std::cos(crawler_orient + PI/2.0) * v_linear * dt;
    crawler_pose_y -= std::cos(crawler_orient + PI/2.0) * v_linear * dt;

    sprintf (ros_info_str, "orient = %f.\t pose_x= %f,\t pose_y=%f.", crawler_orient, crawler_pose_x, crawler_pose_y);
    ROS_INFO ("[ML]%s", ros_info_str);

    geometry_msgs::TwistStamped pose;
    pose.header.stamp = current_time;
    pose.header.frame_id = "crawler_pose";
    pose.twist.linear.x = crawler_pose_x;
    pose.twist.linear.y = crawler_pose_y;
    pose.twist.linear.z = crawler_pose_z;

    pose.twist.angular.x = 0;
    pose.twist.angular.y = 0;
    pose.twist.angular.z = crawler_orient;

    geometry_msgs::Quaternion pose_quat = tf::createQuaternionMsgFromYaw(crawler_orient);
    
    pose_pub.publish(pose);

    x = crawler_pose_x;
    y = crawler_pose_y;

    th = crawler_orient;

  //     orient_ = std::fmod(orient_ + ang_vel_ * dt, 2*PI);
  // pos_.rx() += std::sin(orient_ + PI/2.0) * lin_vel_ * dt;
  // pos_.ry() += std::cos(orient_ + PI/2.0) * lin_vel_ * dt;

    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time();
    marker.ns = "crawler";
    marker.id = 0;
    // marker.type = visualization_msgs::Marker::ARROW;
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::Marker::ADD;
    // VO_TEST // marker.pose.position = visual_odometry.pose.position;
    marker.pose.position.x = crawler_pose_x;
    marker.pose.position.y = crawler_pose_y;
    marker.pose.position.z = 0;
    // VO_TEST // marker.pose.orientation = visual_odometry.pose.orientation;
    marker.pose.orientation = pose_quat;

    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 1;
    marker.color.a = 0.95; // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    //only if using a MESH_RESOURCE marker type:
    marker.mesh_resource = "package://crawler_odometry/meshes/IWAMPV2.0_simple_bin.STL";
    marker.mesh_use_embedded_materials = true;
    crawler_vis_pub.publish( marker );

    // pub wingbay
    visualization_msgs::Marker wingbay;
    wingbay.header.frame_id = "base_link";
    wingbay.header.stamp = ros::Time();
    wingbay.ns = "crawler";
    wingbay.id = 0;
    // wingbay.type = visualization_msgs::wingbay::ARROW;
    wingbay.type = visualization_msgs::Marker::MESH_RESOURCE;
    wingbay.action = visualization_msgs::Marker::ADD;
    wingbay.pose.position.x = 0;
    wingbay.pose.position.y = 0;
    wingbay.pose.position.z = 0;
    wingbay.pose.orientation.x = 0;
    wingbay.pose.orientation.y = 0;
    wingbay.pose.orientation.z = 0;
    wingbay.pose.orientation.w = 0;

    wingbay.scale.x = 1;
    wingbay.scale.y = 1;
    wingbay.scale.z = 1;
    wingbay.color.a = 0.6; // Don't forget to set the alpha!
    wingbay.color.r = 0.5;
    wingbay.color.g = 0.7;
    wingbay.color.b = 0.0;
    //only if using a MESH_RESOURCE wingbay type:
    wingbay.mesh_resource = "package://crawler_odometry/meshes/34WingBay_lower_basement.STL";
    wingbay.mesh_use_embedded_materials = true;
    wingbay_vis_pub.publish( wingbay );


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
    odom.header.frame_id = "base_link";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "odom";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(odom);

    // Save current to last
    last_time = current_time;

    cmd_vel_last_time = cmd_vel_current_time; //save the old time;

    visual_heading_last_time = visual_heading_current_time; //save the old time;
    visual_heading_last_Yaw = visual_heading_current_Yaw; //save the Yaw;


    ROS_INFO (" ");

    r.sleep();
  }
}
