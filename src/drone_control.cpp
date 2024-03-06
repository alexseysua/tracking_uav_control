#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <cmath>
#include <Eigen/Dense>
#include "util.cpp"

#define X_AXIS 1
#define Y_AXIS 0
#define Z_AXIS 0
#define CIRCLE 0
#define YAW 0
#define PITCH 0

mavros_msgs::State current_state;
Eigen::Quaternionf orientation;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

geometry_msgs::Point current_pos;
void pos_cb(const nav_msgs::Odometry::ConstPtr& msg){
    current_pos.x = msg->pose.pose.position.x;
    current_pos.y = msg->pose.pose.position.y;
    current_pos.z = msg->pose.pose.position.z;

    orientation = Eigen::Quaternionf( msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z );
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "drone_control");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("/uav0/mavros/state", 10, state_cb);
    ros::Subscriber pos_sub = nh.subscribe<nav_msgs::Odometry>
            ("/uav0/mavros/global_position/local", 10, pos_cb);

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("/uav0/mavros/setpoint_position/local", 10);
    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::Twist>
            ("/uav0/mavros/setpoint_velocity/cmd_vel_unstamped", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("/uav0/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("/uav0/mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    int ckey = 0, old_ckey = 0;

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 5;

    geometry_msgs::Twist vel;
    vel.linear.x = 0;
    vel.linear.y = 0;
    vel.linear.z = 0.5;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        //local_pos_pub.publish(pose);
        local_vel_pub.publish(vel);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

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
        
        //local_pos_pub.publish(pose);
        local_vel_pub.publish(vel);
        
        if(current_state.armed && current_state.mode == "OFFBOARD"){
            break;
        }

        ros::spinOnce();
        rate.sleep();
    }


    float height = 5, radius = 40, pg_z = 1.6, h = 0, vel_y = 2;
    int flag = 1;

    Eigen::Vector3f euler;
    float yaw, pitch, roll;

    for(float step = 0; step < 8000;)
    {
        euler = Quaternion2Euler(orientation);
        yaw = euler.z();
        pitch = euler.y();
        roll = euler.x();

#if YAW

        vel.linear.z = pg_z * ( height - current_pos.z );
        if( current_pos.z > height - 0.2)
        {
            if( rad2Deg*yaw >= 30 )
            {
                flag = -1;
            }
            if( rad2Deg*yaw <= -30 )
            {
                flag = 1;
            }
            if(flag == 1)
            {
                vel.angular.z = 0.05;
            }
            if(flag == -1)
            {
                vel.angular.z = -0.05;
            }
        }


#endif

#if PITCH

        vel.linear.z = pg_z * ( height - current_pos.z );
        if( current_pos.z > height - 0.2)
        {
            if( rad2Deg*pitch >= 30 )
            {
                flag = -1;
            }
            if( rad2Deg*pitch <= -30 )
            {
                flag = 1;
            }
            if(flag == 1)
            {
                vel.angular.y = 0.2;
            }
            if(flag == -1)
            {
                vel.angular.y = -0.2;
            }
        }


#endif

#if CIRCLE
        vel.linear.z = pg_z * ( height - current_pos.z );
        if( current_pos.z > height - 0.2)
        {
            vel.linear.x = 5*cos(0.2*step); // 3, 0.2 | 2, 0.4/3 | 1.5, 0.1
            vel.linear.y = 5*sin(0.2*step); 

            vel.angular.z = 0.05 * (90*Deg2Rad - yaw);
            if(vel.angular.z > 0.02)
            {
                vel.angular.z = 0.02;
            }

            std::cout << "yaw angle: " << rad2Deg*yaw << std::endl;
            std::cout << "yaw velocity angle: " << vel.angular.z << std::endl;
        }
#endif


#if X_AXIS
        ckey = getch();
        if(ckey != 0)
        {
            old_ckey = ckey;
        }

        height = 4;
        vel.linear.z = pg_z * ( height - current_pos.z );

        if( current_pos.z > height - 0.3)
        {
            /*
            if( current_pos.x <= -7 )
            {
                flag = 1;
            }
            if( current_pos.x >= 7 )
            {
                flag = -1;
            }
            if(flag == 1)
            {
                vel.linear.x = vel_y;
            }
            if(flag == -1)
            {
                vel.linear.x = -vel_y;
            }
            */
            vel.angular.z = 0.2 * (90 - rad2Deg*yaw);

            std::cout << "yaw angle: " << rad2Deg*yaw << std::endl;
            std::cout << "yaw velocity angle: " << vel.angular.z << std::endl;
        }

        if(abs(90 - rad2Deg*yaw) < 3 && old_ckey != 0 && old_ckey == 115) // key (s)
        {
            vel.linear.x = 3;
        }
#endif

#if Y_AXIS
        ckey = getch();
        if(ckey != 0)
        {
            old_ckey = ckey;
        }

        vel.linear.z = pg_z * ( height - current_pos.z );
        if( current_pos.z > height - 0.3)
        {
            /*
            if( current_pos.y <= -5 )
            {
                flag = 1;
            }
            if( current_pos.y >= 5 )
            {
                flag = -1;
            }
            if(flag == 1)
            {
                vel.linear.y = vel_y;
            }
            if(flag == -1)
            {
                vel.linear.y = -vel_y;
            }
            */
           if(abs(0 - abs(rad2Deg*yaw)) < 3 && old_ckey != 0 && old_ckey == 115) // key (s)
            {
                vel.linear.y = 6;
            }
        }

        float yawCmd = 0.2 * (0 - abs(yaw));
        vel.angular.z = 0.2 * (0 - yaw); //std::copysign(yawCmd, yaw);

        std::cout << "yaw angle: " << rad2Deg*yaw << std::endl;
        std::cout << "yaw velocity angle: " << vel.angular.z << std::endl;
#endif

#if Z_AXIS
        if( current_pos.z <= 0.7 )
        {
            flag = 1;
        }
        if( current_pos.z >= 5 )
        {
            flag = -1;
        }
        if(flag == 1)
        {
            vel.linear.z = vel_y;
        }
        if(flag == -1)
        {
            vel.linear.z = -vel_y;
        }
#endif


        //local_pos_pub.publish(pose);
        local_vel_pub.publish(vel);

        ros::spinOnce();
        rate.sleep();
        step += 0.1;
    }

    return 0;
}
