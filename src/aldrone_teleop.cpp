/*
 * This program is aiming to provide an interface for SONY PS3-like controller for controlling ArDrone in ROS.
 *
 * mostly inpired by turtlesim/joy tutorial.
 * Author: ibrahim Eser <ibrahimeser@gmx.com.tr>
 */

#include "aldrone_teleop.h"


AldroneTeleop::AldroneTeleop()
{

//TODO: use ros::ServiceClient serviceflattrim

    nh_.param<int>("axis_x", axis_x, 1);
    nh_.param<int>("axis_y", axis_y, 0);
    nh_.param<int>("axis_z", axis_z, 3);
    nh_.param<int>("axis_yaw", axis_yaw, 2);
    nh_.param<double>("scale_x", scale_x, 1.0);
    nh_.param<double>("scale_y", scale_y, 1.0);
    nh_.param<double>("scale_z", scale_z, 1.0);
    nh_.param<double>("scale_yaw", scale_yaw, 1.0);
    nh_.param<int>("btn_dead_man", btn_dead_man, 6);
    nh_.param<int>("btn_emergency", btn_emergency, 7);
    nh_.param<int>("btn_cam_toggle", btn_cam_toggle, 8);

    sub_joy = nh_.subscribe<sensor_msgs::Joy> ("joy", 10, &AldroneTeleop::joyCallBack, this);
    pub_vel = nh_.advertise<geometry_msgs::Twist> ("ardrone/cmd_vel",1);

    pub_takeoff       = nh_.advertise<std_msgs::Empty>("/ardrone/takeoff",1);
    pub_land          = nh_.advertise<std_msgs::Empty>("/ardrone/land",1);
    pub_toggle_state  = nh_.advertise<std_msgs::Empty>("/ardrone/reset",1);
    pub_vel           = nh_.advertise<geometry_msgs::Twist>("/cmd_vel",1);
    srv_cl_cam        = nh_.serviceClient<std_srvs::Empty>("/ardrone/togglecam",1);

    twist.linear.x = twist.linear.y = twist.linear.z = 0.0;
    twist.angular.x = twist.angular.y = twist.angular.z = 0.0;

    is_flying = false;
    got_first_joy_msg = false;
    toggle_pressed_in_last_msg = false;
    cam_toggle_pressed_in_last_msg = false;

    ROS_INFO("Teleoperation Node <aldrone_teleop> started");
}


void
AldroneTeleop::joyCallBack (const sensor_msgs::Joy::ConstPtr& joy)
{
    if (!got_first_joy_msg) {
        ROS_INFO("Found joystick with %zu buttons and %zu axes", joy->buttons.size(), joy->axes.size());
    }

    got_first_joy_msg = true;
    ROS_INFO ("In AldroneTeleop::joyCallBack()");

    //TODO: Use params for btns instaed numbers
    bool dead_man_pressed = joy->buttons.at(btn_dead_man);
    ROS_INFO("L1 was pressed,");
    bool emergency_toggle_pressed = joy->buttons.at(btn_emergency);
    bool cam_toggle_pressed = joy->buttons.at(btn_cam_toggle);
    ROS_INFO("Btn#8  was pressed,");

    if (!is_flying && dead_man_pressed){
        ROS_INFO("L1 was pressed, Taking off!");
        pub_takeoff.publish(std_msgs::Empty());
        is_flying = true;
    }

    if (is_flying && !dead_man_pressed){
        ROS_INFO("L1 was released, Landing");
        pub_land.publish(std_msgs::Empty());
        is_flying = false;
    }

    // toggle only once!
    if (!toggle_pressed_in_last_msg && emergency_toggle_pressed){
        ROS_INFO("Changing emergency status");
        pub_toggle_state.publish(std_msgs::Empty());
    }

    toggle_pressed_in_last_msg = emergency_toggle_pressed;

    if (!cam_toggle_pressed_in_last_msg && cam_toggle_pressed){
        ROS_INFO("Trying to Toggle Camera Source");
    if (!srv_cl_cam.call(srv_empty))
        ROS_INFO("Failed to toggle Camera");
    else
        ROS_INFO("Camera Source Toggled");
    }
    cam_toggle_pressed_in_last_msg = cam_toggle_pressed;


    if (is_flying && dead_man_pressed){
        twist.linear.x = joy->axes[axis_x];
        twist.linear.y = joy->axes[axis_y];
        twist.linear.z = joy->axes[axis_z];
        twist.angular.z = joy->axes[axis_yaw];
        pub_vel.publish(twist);
    }

}

int main (int argc, char** argv)
{
    ros::init (argc, argv, "aldrone_teleop");
    AldroneTeleop aldrone_teleop;
    ros::NodeHandle nodeHandle;

    ROS_INFO("Teleoperation Node <aldrone_teleop> started");
    ROS_INFO("Press and hold L1 for takeoff");
    ROS_INFO("Press L2 to toggle emergency-state");
    ROS_INFO("Press 'select' to choose camera");

    ros::Rate spinRate(SPIN_RATE);

    while (nodeHandle.ok()) {
        ros::spinOnce();
        spinRate.sleep();
    }
}
