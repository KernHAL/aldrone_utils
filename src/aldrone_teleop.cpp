/*
 * This program is aiming to provide an interface for SONY PS3-like controller for controlling ArDrone in ROS.
 *
 * mostly inpired by turtlesim/joy tutorial.
 * Author: ibrahim Eser <ibrahimeser@gmx.com.tr>
 */

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>

class AldroneTeleop
{
    public:
        AldroneTeleop();

    private:
        bool got_first_joy_msg;
        bool is_flying;
        bool toggle_pressed_in_last_msg;
        bool cam_toggle_pressed_in_last_msg;

        int axis_roll;
        int axis_pitch;
        int axis_yaw;
        int axis_z;

        double scale_roll;
        double scale_pitch;
        double scale_yaw;
        double scale_z;

        void joyCallBack (const sensor_msgs::Joy::ConstPtr& joy);

        ros::NodeHandle nh_;

        ros::Publisher pub_vel;
        ros::Publisher pub_takeoff;
        ros::Publisher pub_land;
        ros::Publisher pub_toggle_state;
        ros::Subscriber sub_joy;

        ros::ServiceClient srv_cl_cam;
};

AldroneTeleop::AldroneTeleop()
{
    nh_.param ("axis_roll", axis_roll, axis_roll);
    nh_.param ("axis_pitch", axis_pitch, axis_pitch);
    nh_.param ("axis_yaw", axis_yaw, axis_yaw);
    nh_.param ("axis_z", axis_z, axis_z);
    nh_.param ("scale_roll", scale_roll, scale_roll);
    nh_.param ("scale_z", scale_z, scale_z);

    sub_joy = nh_.subscribe<sensor_msgs::Joy> ("joy", 10, &AldroneTeleop::joyCallBack, this);
    pub_vel = nh_.advertise<geometry_msgs::Twist> ("ardrone/cmd_vel",1);

    pub_takeoff       = nh_.advertise<std_msgs::Empty>("/ardrone/takeoff",1);
    pub_land          = nh_.advertise<std_msgs::Empty>("/ardrone/land",1);
    pub_toggle_state  = nh_.advertise<std_msgs::Empty>("/ardrone/reset",1);
    pub_vel           = nh_.advertise<geometry_msgs::Twist>("/cmd_vel",1);

    srv_cl_cam        = nh_.serviceClient<std_srvs::Empty>("/ardrone/togglecam",1);

    is_flying = false;
    got_first_joy_msg = false;
    toggle_pressed_in_last_msg = false;
    cam_toggle_pressed_in_last_msg = false;
}


void AldroneTeleop::joyCallBack (const sensor_msgs::Joy::ConstPtr& joy)
{
    geometry_msgs::Twist twist;
    twist.linear.x = scale_roll * joy->axes[axis_roll];
//    twist.linear.y = scale_pitch* joy->axes[axis_pitch];
//    twist.linear.z = scale_yaw* joy->axes[axis_yaw];
    twist.angular.z = scale_z * joy->axes[axis_z];
    pub_vel.publish(twist);

}

int main (int argc, char** argv)
{
    ros::init (argc, argv, "aldrone_teleop");
    AldroneTeleop aldrone_teleop;

    ROS_INFO("Teleoperation Node <aldrone_teleop> started");
    ROS_INFO("Press L1 to toggle emergency-state");
    ROS_INFO("Press and hold L2 for takeoff");
    ROS_INFO("Press 'select' to choose camera");


        ROS_INFO("befor ros::spin()");
        ros::spin();
        ROS_INFO("after ros::spin()");


}
