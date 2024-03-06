/**************************************
 * stack and tested in Gazebo SITL
 **************************************/
#ifndef FWCONTROL_HPP
#define FWCONTROL_HPP

#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/VFR_HUD.h>
#include <nav_msgs/Odometry.h>

using namespace std;

class FwControl
{
private:
    ros::NodeHandle nh;
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate = 5.0;
    ros::Subscriber state_sub;
    ros::Subscriber global_pos_sub;
    ros::Subscriber hud_sub;
    ros::Subscriber fw_pose_sub;
    ros::Publisher vel_pub;
    ros::Publisher localPos_pub;
    ros::Publisher att_pub;
    ros::ServiceClient arming_client;
    ros::ServiceClient set_mode_client;
    ros::ServiceClient takeoff_client;
    ros::Time last_request;
    
    mavros_msgs::SetMode offb_set_mode;
    mavros_msgs::CommandBool cmd_arm;
    mavros_msgs::State current_state;
    mavros_msgs::CommandTOL cmd_takeoff;
    sensor_msgs::NavSatFix current_lla;
    geometry_msgs::TwistStamped cmd_vel;
    geometry_msgs::PoseStamped cmd_pos;
    mavros_msgs::AttitudeTarget cmd_att;
    Eigen::Quaternionf quat_local;
    Eigen::Vector3f euler_local;

    std::string controlMode = "OFFBOARD";
    bool isTakeoff = 0;
    mavros_msgs::VFR_HUD current_hud;


public: 
    FwControl();
    ~FwControl();
    bool CheckFCUConnection();
    bool Initialize();
    void SwitchMode(std::string flightMode);
    void SwitchFlightMode(std::string flightMode);
    void Takeoff();
    void TakeoffPos();
    void TakeoffVel();
    void TrackCircle(std::vector<float> ctr, float radius, float freq);
    void GoStraight();
    void GoStraightVel();
    void StraightUpNDown();
    void Loiter();

    void getCurrentState(const mavros_msgs::State::ConstPtr& state);
    void getGlobalLLA(const sensor_msgs::NavSatFix::ConstPtr& lla);
    void hud_cb(const mavros_msgs::VFR_HUD::ConstPtr& msg);
    void getFwPose(const nav_msgs::Odometry::ConstPtr& pose);
};

#endif