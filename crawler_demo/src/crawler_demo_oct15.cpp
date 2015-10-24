#include <ros/ros.h>
//#include <geometry_msgs/Twist.h>#include <turtlesim/Velocity.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <math.h>  

// Crawler Messages
#include <crawler_msgs/JointCmd.h>

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

class  CrawlerDemo
{
public:
  CrawlerDemo();
  ros::NodeHandle nh_;

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  

  int linear_, angular_, slowMo_;
  
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;
  
  ros::Publisher joint_cmd_pub_;
  
};
  CrawlerDemo():
  linear_(1),angular_(0),slowMo_(2)
{

  nh_.param("axis_linear", linear_, linear_);
  nh_.param("axis_angular", angular_, angular_);
  nh_.param("axis_slowMo", slowMo_, slowMo_);

  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);

  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10,  CrawlerDemo::joyCallback, this);
  
  joint_cmd_pub_ = nh_.advertise<crawler_msgs::JointCmd>("crawler/joint_cmd", 10);

}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "crawler_demo");
  CrawlerDemo crawler_demo;

  ros::Rate r(10); // 10Hz

  while(nh_.ok()){
    ros::spinOnce(); 
  }

  //ros::spin();
}

