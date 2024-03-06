/**************************************
 * stack and tested in Gazebo SITL
 **************************************/
#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <mavros_msgs/VFR_HUD.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "util.cpp"

#define cmd_vel 0
#define cmd_att 1

using namespace std;
using namespace Eigen;

typedef struct 
{
    double thrust;
    double rate_x;
    double rate_y;
    double rate_z;
    double x;
    double y;
    double z;
    double w;
}attitudeCmd;

attitudeCmd cmd;
Quaternionf quat;
Vector3f rpy;

mavros_msgs::State current_state;
sensor_msgs::NavSatFix current_globalPos;
mavros_msgs::VFR_HUD current_hud;
nav_msgs::Odometry odom;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void globalPos_cb(const sensor_msgs::NavSatFix::ConstPtr& msg){
    current_globalPos = *msg;
}

void hud_cb(const mavros_msgs::VFR_HUD::ConstPtr& msg)
{
    current_hud = *msg;
}

void odom_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
    odom = *msg;
    quat.x() = odom.pose.pose.orientation.x;
    quat.y() = odom.pose.pose.orientation.y;
    quat.z() = odom.pose.pose.orientation.z;
    quat.w() = odom.pose.pose.orientation.w;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "fw_takeoff");
    ros::NodeHandle nh;
    /*** Subscriber ***/
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("/uav0/mavros/state", 10, state_cb);
    ros::Subscriber global_pos_sub = nh.subscribe<sensor_msgs::NavSatFix>
            ("/uav0/mavros/global_position/global", 10, globalPos_cb);
    ros::Subscriber hud_sub = nh.subscribe<mavros_msgs::VFR_HUD>
            ("/uav0/mavros/vfr_hud", 10, hud_cb);
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>
            ("/uav0/mavros/local_position/odom", 10, odom_cb);

    /*** Publisher ***/
    ros::Publisher init_vel_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("/uav0/mavros/setpoint_velocity/cmd_vel", 10);
    ros::Publisher att_pub = nh.advertise<mavros_msgs::AttitudeTarget>
            ("/uav0/mavros/setpoint_raw/attitude", 10);

    /*** ServiceClient ***/
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("/uav0/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("/uav0/mavros/set_mode");
    ros::ServiceClient takeoff_client = nh.serviceClient<mavros_msgs::CommandTOL>
            ("/uav0/mavros/cmd/takeoff");
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);
   
    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    
    //send a few setpoints before starting
    geometry_msgs::TwistStamped vel;
    vel.twist.linear.x = 10.0;
    for (int i = 100; ros::ok() && i > 0; i--){
        init_vel_pub.publish(vel);
        ROS_INFO("Initializing");
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    mavros_msgs::CommandTOL takeoff_cmd;
    takeoff_cmd.request.latitude = current_globalPos.latitude;
    takeoff_cmd.request.longitude = current_globalPos.longitude + 0.008;
    takeoff_cmd.request.altitude = current_globalPos.altitude + 100;
    bool isTakeoff = 0;
    int takeoff_count = 0;
    
    ros::Time last_request = ros::Time::now();
    while(ros::ok())
    {
        if( !current_state.armed &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( arming_client.call(arm_cmd) &&
                arm_cmd.response.success){
                ROS_INFO("Vehicle armed");
            }
            last_request = ros::Time::now();
        } else {
            if( current_state.mode != "OFFBOARD" &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( set_mode_client.call(offb_set_mode) &&
                    offb_set_mode.response.mode_sent ){
                    ROS_INFO("Offboard enabled");
                }
                last_request = ros::Time::now();
            }
        }
        
        if(current_state.armed && current_state.mode == "OFFBOARD"){
            break;
        }

        init_vel_pub.publish(vel);
        
        ros::spinOnce();
        rate.sleep();
    }

#if cmd_att
    mavros_msgs::AttitudeTarget set_att;
    double spec[] = {32.92, 500};  // units: m/s, m
    double error[] = {0, 0};
    while(ros::ok())
    {
        rpy = Quaternion2Euler(quat);  // ENU
        std::cout << "Quaternion0" << std::endl << quat.coeffs() << std::endl;
        cout << "rpy: " << rad2Deg*rpy[0] << ", " << rad2Deg*rpy[1] << ", " << rad2Deg*rpy[2] << endl;

        Quaternionf q;
        q = AngleAxisf(rpy[2], Vector3f::UnitZ())
            * AngleAxisf(Deg2Rad*rpy[1], Vector3f::UnitY())
            * AngleAxisf(rpy[0], Vector3f::UnitX());
        std::cout << "Quaternion1" << std::endl << q.coeffs() << std::endl;
        rpy = Quaternion2Euler(q);
        cout << "rpy: " << rad2Deg*rpy[0] << ", " << rad2Deg*rpy[1] << ", " << rad2Deg*rpy[2] << endl;










        error[0] = spec[0] - current_hud.groundspeed;
        error[1] = spec[1] - odom.pose.pose.position.z;


        double pitch = - 0.1 * error[1];
        pitch = (abs(pitch) > 45)?copysign(45, pitch):pitch;
        tf2::Quaternion qt;
        qt.setRPY(0.0, pitch, 0.0);
        cout << "cmd pitch: " << pitch << endl;
        cout << "cmd q: " << qt.x() << ", " << qt.y() << ", " << qt.z() << ", " << qt.w() << endl; 



        cmd.thrust = 0.7; //0.4 + 0.08 * error[0];

        
        /*
        cmd.x = qt.x();
        cmd.y = qt.y();
        cmd.z = qt.z();
        cmd.w = qt.w();
        */

        
        cmd.rate_x = 0.5 * (Deg2Rad*0 - rpy[0]);
        cmd.rate_y = 3.2 * (-Deg2Rad*30 - rpy[1]);
        cmd.rate_z = 0.5 * (Deg2Rad*0 - rpy[2]);
        
        //cout << "cmd y: " << cmd.rate_y << endl;

        cmd.thrust = (abs(cmd.thrust) > 0.85)?copysign(0.85, cmd.thrust):cmd.thrust;
        
        cmd.rate_x = (abs(cmd.rate_x) > 1)?copysign(1, cmd.rate_x):cmd.rate_x;
        cmd.rate_y = (abs(cmd.rate_y) > 1)?copysign(1, cmd.rate_y):cmd.rate_y;
        cmd.rate_z = (abs(cmd.rate_z) > 1)?copysign(1, cmd.rate_z):cmd.rate_z;
        

        set_att.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE | 
                            mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE | 
                            mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;

        set_att.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;
        set_att.thrust = 0.2; //cmd.thrust;
        
        set_att.body_rate.x = cmd.rate_x;
        set_att.body_rate.y = cmd.rate_y;
        set_att.body_rate.z = cmd.rate_z;
        
        /*
        set_att.orientation.x = cmd.x;
        set_att.orientation.y = cmd.y;
        set_att.orientation.z = cmd.z;
        set_att.orientation.w = cmd.w;
        */

        cout << "body_rate: " << set_att.body_rate.x << ", " << set_att.body_rate.y << ", " << set_att.body_rate.z << endl;
        cout << "thrust: " << set_att.thrust << endl << "--------------------------------------" << endl;

        att_pub.publish(set_att);
        ros::spinOnce();
        rate.sleep();
    }
#endif


# if cmd_vel
    geometry_msgs::TwistStamped set_vel;
    double spec[] = {33, 1500};  // units: m/s, m
    double error[] = {0, 0};
    while(ros::ok())
    {
        double cmd[] = {0, 0, 0};
        error[0] = spec[0] - current_hud.groundspeed;
        error[1] = spec[1] - odom.pose.pose.position.z;

        cmd[0] = 60 + 0.8 * error[0];
        cmd[1] = 40 + 0.8 * error[1];

        set_vel.twist.linear.x = cmd[0];
        set_vel.twist.linear.y = 0; 
        set_vel.twist.linear.z = cmd[1];

        cout << "vel: " << set_vel.twist.linear.x << ", " << set_vel.twist.linear.y << ", " << set_vel.twist.linear.z << endl;

        init_vel_pub.publish(set_vel);
        ros::spinOnce();
        rate.sleep();
    }
#endif



    return 0;
}