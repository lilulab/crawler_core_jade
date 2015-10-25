#include <ros/ros.h>
//#include <geometry_msgs/Twist.h>#include <turtlesim/Velocity.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <math.h>  

// Crawler Messages
#include "crawler_msgs/JointCmd.h"

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

class TeleopTurtle
{
public:
  TeleopTurtle();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  
  ros::NodeHandle nh_;

  int linear_, angular_, slowMo_;
  int button_nos_;
  
  int arm_yaw_, arm_pic_, arm_pmt_ext_, arm_pmt_cnt_;
  
  int trs_bas_in_, trs_bas_out_;
  int cbw_lft_in_, cbw_lft_out_;
  int cbw_rgt_in_, cbw_rgt_out_;
  
  int wrt_rol_lft_, wrt_rol_rgt_;
  int wrt_pic_up_, wrt_pic_down_;
  
  int but_stop_all_;
   
  double l_scale_;
  double a_scale_;
  double slowMo_scale_;
  int count;

  int non_zero_total_count;
  
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;
  
  ros::Publisher joint_cmd_pub_;
  
};


TeleopTurtle::TeleopTurtle():
  linear_(1),angular_(0),slowMo_(2),
  l_scale_(1),a_scale_(1),slowMo_scale_(0.2),
  button_nos_(4),
  arm_yaw_(3), arm_pic_(4), arm_pmt_ext_(5), arm_pmt_cnt_(5),
  trs_bas_in_(14), trs_bas_out_(13),
  cbw_lft_in_(11), cbw_lft_out_(12),
  cbw_rgt_in_(11), cbw_rgt_out_(12),
  wrt_rol_lft_(2), wrt_rol_rgt_(1),
  wrt_pic_up_(3), wrt_pic_down_(0),
  but_stop_all_(8)
{

  nh_.param("axis_linear", linear_, linear_);
  nh_.param("axis_angular", angular_, angular_);
  nh_.param("axis_slowMo", slowMo_, slowMo_);
  
  nh_.param("button_NOS", button_nos_, button_nos_);
  
  nh_.param("scale_linear", l_scale_, l_scale_);
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_slowMo", slowMo_scale_, slowMo_scale_);

  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);

  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopTurtle::joyCallback, this);
  
  joint_cmd_pub_ = nh_.advertise<crawler_msgs::JointCmd>("crawler/joint_cmd", 10);

}

void TeleopTurtle::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    // vel pub data
    geometry_msgs::Twist vel;

    // joint cmd vel data
    crawler_msgs::JointCmd joint_cmd;

    // Old ROS
    //vel.angular = a_scale_*joy->axes[angular_];
    //vel.linear = l_scale_*joy->axes[linear_];
    if (joy != NULL) {
        // ROS > Hydro

        // LS for drive the base motion.

        vel.linear.x = joy->axes[linear_];
        vel.angular.z = joy->axes[angular_]; 

        ROS_INFO("Vx(0)     = %f",vel.linear.x);
        ROS_INFO("Vz(0)     = %f",vel.angular.z);       

        // Dead zone enforcement!
        if (fabs(vel.linear.x)< 0.2) {
          vel.linear.x = 0.0;
        } else {
          vel.linear.x = vel.linear.x;
          //vel.linear.x -= 0.2 * (vel.linear.x/vel.linear.x) ; 
        }

        if (fabs(vel.angular.z)< 0.2) {
          vel.angular.z = 0.0;
        } else {
          vel.angular.z = vel.angular.z;
          //vel.angular.z -= 0.2 * (vel.angular.z/vel.angular.z); 
        }

        ROS_INFO("Vx(t)     = %f",vel.linear.x);
        ROS_INFO("Vz(t)     = %f",vel.angular.z);

        //vel.linear.x = joy->axes[linear_];
        //vel.angular.z = joy->axes[angular_];


        vel.linear.x = l_scale_ * vel.linear.x; //LS
        vel.angular.z = a_scale_ * vel.angular.z; //LS
        // Speed slowMo
        float slowMo;
        slowMo = joy->axes[slowMo_]; // RT
        float nos;
        //nos = (float)pow(0,joy->buttons[button_nos_]);
        nos = (float)(joy->buttons[button_nos_]+1);
        ROS_INFO("nos  = %f",nos);

        // Deadzone
        //if (vel.linear.x < 0.2f) { vel.linear.x = 0.0f};
        //if (vel.linear.z < 0.2f) { vel.linear.z = 0.0f};

        // Set scale and slowMo
        slowMo = (-slowMo+1)/2; //slowMo raw [none,pressed]= [1,-1], remap to [0,1];
        slowMo = (slowMo-1)*(slowMo_scale_-1)+slowMo_scale_; //remap to [1,slowMo_scale_]

        vel.linear.x = vel.linear.x * l_scale_ * slowMo * nos;
        vel.angular.z = vel.angular.z * a_scale_ * slowMo * nos;

        // Debug
        ROS_INFO("slowMo  = %f",slowMo);
        ROS_INFO("Vx     = %f",vel.linear.x);
        ROS_INFO("Vz     = %f",vel.angular.z);

        // cmd vel update 
        joint_cmd.header.seq = count;
        joint_cmd.header.stamp = ros::Time::now();

        // Wheel cmd
        joint_cmd.jointCmdVel[MMC_WhlLft_JointID] = vel.linear.x * 4 - vel.angular.z * 4;
        joint_cmd.jointCmdVel[MMC_WhlRgt_JointID] = -vel.linear.x * 4 - vel.angular.z * 4;

        // Arm P Y
        joint_cmd.jointCmdVel[MMC_ArmYaw_JointID] = joy->axes[arm_yaw_] * 6 * slowMo * nos;
        joint_cmd.jointCmdVel[MMC_ArmPic_JointID] = joy->axes[arm_pic_] * -0.75  * slowMo * nos;

        /*
        // RB for trigger mode
        if (joy->buttons[arm_pmt_cnt_] == 0) {
        joint_cmd.jointCmdVel[MMC_ArmPmt_JointID] = (joy->axes[arm_pmt_ext_]-1)/2;
        } else {
        joint_cmd.jointCmdVel[MMC_ArmPmt_JointID] = (joy->axes[arm_pmt_ext_]-1)/2 * -1;
        }
        */

        // AmrPmt
        if (joy->axes[arm_pmt_ext_] !=0) {
            joint_cmd.jointCmdVel[MMC_ArmPmt_JointID] = (joy->axes[arm_pmt_ext_]-1)/2 * -8  * slowMo * nos;
        }
        
        if (joy->buttons[arm_pmt_cnt_] == 1) {
            joint_cmd.jointCmdVel[MMC_ArmPmt_JointID] = -8  * slowMo * nos;
        }  

        // Trans Base
        joint_cmd.jointCmdVel[MMC_TrsBas_JointID] = (joy->buttons[trs_bas_in_] * 10 + joy->buttons[trs_bas_out_] * -10) * slowMo * nos;
                                                
        // CBW
        joint_cmd.jointCmdVel[MMC_CbwLft_JointID] = (joy->buttons[cbw_lft_in_] * 10 + joy->buttons[cbw_lft_out_] * -10) * slowMo * nos ;
        joint_cmd.jointCmdVel[MMC_CbwRgt_JointID] = (joy->buttons[cbw_rgt_in_] * 10 + joy->buttons[cbw_rgt_out_] * -10) * slowMo * nos;  

        // Wrist  
        joint_cmd.jointCmdVel[MMC_WrtRol_JointID] = (float) 0.1 * (joy->buttons[wrt_rol_lft_] * 4 + joy->buttons[wrt_rol_rgt_] * -4) * slowMo * nos;
        joint_cmd.jointCmdVel[MMC_WrtPic_JointID] = (float) 0.1 * (joy->buttons[wrt_pic_up_] * 6 + joy->buttons[wrt_pic_down_] * -6) * slowMo * nos;
    } //End if
    

        // E-Stop
        double is_cmd_vel_non_zero = 0;
        for (int i=0; i<10; i++) {
              is_cmd_vel_non_zero += fabs(joint_cmd.jointCmdVel[i]);
          if (joy->buttons[but_stop_all_] == 1) {
              
              joint_cmd.jointCmdVel[i] = 0;
          }

        } // end for
        
        // check if all of joy axis is non zero
        double is_joy_non_zero = 0;

        //ROS_INFO("size_axis = %f",sizeof(joy->axes));
        //ROS_INFO("size_buts = %f",sizeof(joy->buttons));

        for (int i=0; i<6; i++) {
          is_joy_non_zero += fabs(joy->axes[i]);
          ROS_INFO("joy_non_zero_axis = %f",is_joy_non_zero);
        }

        is_joy_non_zero -=2.0; // two trigger axis min is 1.0;

        for (int i=0; i<15; i++) {
          is_joy_non_zero += fabs(joy->buttons[i]);
          ROS_INFO("joy_non_zero_but = %f",is_joy_non_zero);
        }        

        //6axis // 15 but

        ROS_INFO("joy_non_zero_sum = %f",is_joy_non_zero);

        if (is_joy_non_zero == 0) {
          non_zero_total_count ++;

        } else {
          non_zero_total_count = 0;
        }

        ROS_INFO("non_zero_total_count = %d",non_zero_total_count);

      // publish only if joy is non zero.
      //if (is_joy_non_zero != 0) {
      if (non_zero_total_count < 5) {
        // Publish vel
        vel_pub_.publish(vel);
        // Publish cmd vel
        joint_cmd_pub_.publish(joint_cmd);
        ++count;
      } // end if

    

}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_turtle");
  TeleopTurtle teleop_turtle;

  ros::spin();
}

