
/*
 * This program is aiming to provide an interface for SONY PS3-like controller
 * for controlling ArDrone in ROS.
 *
 * mostly inpired by turtlesim/joy tutorial.
 * Author: ibrahim Eser <ibrahimeser@gmx.com.tr>
 */

#ifndef __ALDRONE_TELEOP_H
#define __ALDRONE_TELEOP_H

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>

class AldroneTeleop {
 public:
  AldroneTeleop();
  static const int  kSpinRate = 10;

 private:
  bool got_first_joy_msg;
  bool toggle_pressed_in_last_msg;
  bool cam_toggle_pressed_in_last_msg;

  bool is_flying;
  bool dead_man_pressed;

  int axis_x;
  int axis_y;
  int axis_z;
  int axis_yaw;

  double scale_x;
  double scale_y;
  double scale_z;
  double scale_yaw;

  int btn_dead_man;
  int btn_emergency;
  int btn_cam_toggle;

  geometry_msgs::Twist twist;
  std_srvs::Empty srv_empty;

  ros::NodeHandle nh_;
  ros::Publisher pub_vel;
  ros::Publisher pub_takeoff;
  ros::Publisher pub_land;
  ros::Publisher pub_toggle_state;
  ros::Subscriber sub_joy;
  ros::ServiceClient srv_cl_cam;

  void JoyCallBack(const sensor_msgs::Joy::ConstPtr& joy);
};

#endif /* __ALDRONE_TELEOP_H */
