#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <stdio.h>

#include <sstream>

#define  setbit(x, y)  x|=(1<<y)
#define  clrbit(x, y)  x&=~(1<<y)

//low three bit as lidar warn flag
//                     left  front right
#define  STATUS_A   0x04  // 1 0 0
#define  STATUS_B   0x02  // 0 1 0
#define  STATUS_C   0x01  // 0 0 1
#define  STATUS_D   0x07  // 1 1 1
#define  STATUS_E   0x06  // 1 1 0
#define  STATUS_F   0x03  // 0 1 1
#define  STATUS_G   0x05  // 1 0 1

//global variable
geometry_msgs::Twist twist_cmd;
ros::Publisher twist_pub;

const double warn_range = 0.5;  //warn check distance
double default_period_hz = 10;  //hz
double default_linear_x = 0.5;  // (m/s)
double default_yaw_rate = 0.5;  // rad/s

double range_array[3]; //save three obstacle distances
double des_cmd_array[2]; //save the linear_x and angular_z from des_cmd


void laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{ 
  range_array[0] = 999;
  range_array[1] = 999;
  range_array[2] = 999; //reset the obstacle distances to 999

  for(int i = 45; i < 75; i++) //find the cloest obstacle on right
  {
    if (msg->ranges[i] < range_array[2]) range_array[2] = msg->ranges[i];
    else continue;
  }

  for(int i = 105; i < 165; i++) //find the cloest obstacle ahead
  {
    if (msg->ranges[i] < range_array[1]) range_array[1] = msg->ranges[i];
    else continue;
  }

  for(int i = 195; i < 225; i++) //find the cloest obstacle on left
  {
    if (msg->ranges[i] < range_array[0]) range_array[0] = msg->ranges[i];
    else continue;
  }
  
}

void des_vel_callback(const geometry_msgs::Twist::ConstPtr& msg)
{
  des_cmd_array[0] = msg->linear.x;
  des_cmd_array[1] = msg->angular.z;
}

void publishTwistCmd(double linear_x, double angular_z)
{
    twist_cmd.linear.x = linear_x;
    twist_cmd.linear.y = 0.0;
    twist_cmd.linear.z = 0.0;
 
    twist_cmd.angular.x = 0.0;
    twist_cmd.angular.y = 0.0;
    twist_cmd.angular.z = angular_z;
 
    twist_pub.publish(twist_cmd);
}

void smart_teleoperation(double lidar_l, double lidar_f, double lidar_r)
{
  unsigned char flag = 0;
 
    if(lidar_l < warn_range) setbit(flag, 2);
    else clrbit(flag, 2);
 
    if(lidar_f < warn_range) setbit(flag, 1);
    else clrbit(flag, 1);

    if(lidar_r < warn_range) setbit(flag, 0);
    else clrbit(flag, 0);
    
    ROS_INFO("CheckSonarRange get status:0x%x", flag);
    
    switch(flag)
    {
      case STATUS_A: // left warn,turn right
        publishTwistCmd(0, -default_yaw_rate);
        break;
        
      case STATUS_B: // front warn, left and right ok, compare left and right value to turn
        if(lidar_l > lidar_r)  publishTwistCmd(0, default_yaw_rate);
        else  publishTwistCmd(0, -default_yaw_rate);
        break;
      
      case STATUS_C: // right warn, turn left
        publishTwistCmd(0, default_yaw_rate); 
        break;
        
      case STATUS_D: // left,front,right all warn, turn back
        publishTwistCmd(0, 10*default_yaw_rate);
        break;
        
      case STATUS_E: // left warn, front warn, right ok, turn right
        publishTwistCmd(0, (-default_yaw_rate*2));
        break;
        
      case STATUS_F: // left ok, front warn, right warn, turn left
        publishTwistCmd(0, (default_yaw_rate*2));
        break;
      
      case STATUS_G: // left and right warn, front ok, speed up
        publishTwistCmd(2*default_linear_x, 0);
        break;
        
      default: // no warning, ues the des_cel
        publishTwistCmd(des_cmd_array[0], des_cmd_array[1]);
        break;     
    }
    
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "smart_teleoperation_node");

  ros::NodeHandle n;

  ros::Rate loop_rate(default_period_hz);

  ros::Subscriber sub_laser = n.subscribe("laser_1", 100, laser_callback);
    
  ros::Subscriber sub_des = n.subscribe("des_vel", 100, des_vel_callback);
  
  twist_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  
  while (ros::ok())
  {
    smart_teleoperation(range_array[0], range_array[1], range_array[2]);
    
    ros::spinOnce();

    loop_rate.sleep();

  }

  return 0;
}


