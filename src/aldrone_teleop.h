
/*
 * This program is aiming to provide an interface for SONY PS3-like controller for controlling ArDrone in ROS.
 *
 * mostly inpired by turtlesim/joy tutorial.
 * Author: ibrahim Eser <ibrahimeser@gmx.com.tr>
 */

#ifndef __ALDRONE_TELEOP_H
#define __ALDRONE_TELEOP_H


#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>

#define SPIN_RATE 10

class AldroneTeleop
{
    public:
        AldroneTeleop();

    private:
        bool got_first_joy_msg;
        bool toggle_pressed_in_last_msg;
        bool cam_toggle_pressed_in_last_msg;

        bool is_flying;
        bool dead_man_pressed;

        int axis_roll;
        int axis_pitch;
        int axis_yaw;
        int axis_z;

        double scale_roll;
        double scale_pitch;
        double scale_yaw;
        double scale_z;

        geometry_msgs::Twist twist;
        std_srvs::Empty srv_empty;

        ros::NodeHandle nh_;
        ros::Publisher pub_vel;
        ros::Publisher pub_takeoff;
        ros::Publisher pub_land;
        ros::Publisher pub_toggle_state;
        ros::Subscriber sub_joy;
        ros::ServiceClient srv_cl_cam;

        void joyCallBack (const sensor_msgs::Joy::ConstPtr& joy);
};

#endif /* __ALDRONE_TELEOP_H */
