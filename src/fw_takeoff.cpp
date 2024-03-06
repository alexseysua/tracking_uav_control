/**************************************
 * stack and tested in Gazebo SITL
 **************************************/
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

using namespace std;

mavros_msgs::State current_state;
sensor_msgs::NavSatFix current_globalPos;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void globalPos_cb(const sensor_msgs::NavSatFix::ConstPtr& msg){
    current_globalPos = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fw_takeoff");
    ros::NodeHandle nh;
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("/uav0/mavros/state", 10, state_cb);
    ros::Subscriber global_pos_sub = nh.subscribe<sensor_msgs::NavSatFix>
            ("/uav0/mavros/global_position/global", 10, globalPos_cb);
    ros::Publisher init_vel_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("/uav0/mavros/setpoint_velocity/cmd_vel", 10);
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
    vel.twist.linear.x = 0.01;
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
    bool is_takeoff = 0;
    
    ros::Time last_request = ros::Time::now();
    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
        (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
        if(!is_takeoff){
            if( current_state.mode == "OFFBOARD" && current_state.armed){
            takeoff_client.call(takeoff_cmd);
            ROS_INFO("Takeoff");
            is_takeoff = takeoff_cmd.response.success;
            }
        }

        ROS_INFO("Velocity publish");
        init_vel_pub.publish(vel);
        
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}