#include <iostream>
#include <Eigen/Dense>
#include <cmath>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>

#include "util.cpp"

#define ST_CONSTANT_VEL 0
#define ST_VARIABLE_VEL 1
#define ST_RANDOM_VEL 0
#define CIRCLE_CONSTANT_VEL 0

using namespace std;

geometry_msgs::Point position;
Eigen::Quaternionf q;
Eigen::Vector3f euler;

void position_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
    position.x = msg->pose.pose.position.x;
    position.y = msg->pose.pose.position.y;
    position.z = msg->pose.pose.position.z;

    q.x() = msg->pose.pose.orientation.x;
    q.y() = msg->pose.pose.orientation.y;
    q.z() = msg->pose.pose.orientation.z;
    q.w() = msg->pose.pose.orientation.w;

    euler = Quaternion2Euler(q);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "car_control");
    ros::NodeHandle nh;
    ros::Rate rate(20);
    ros::Subscriber position_sub = nh.subscribe<nav_msgs::Odometry>("/prius/pose_ground_truth", 2, position_cb);
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/prius/cmd_vel", 2);

    geometry_msgs::Point init_pose;
    init_pose.y = 0;

    float vel_desire = 20;
    float distance = 100;
    int flag_yaw = 0, flag_y = 0;

    geometry_msgs::Twist velCmd;
    velCmd.linear.x = vel_desire;
    velCmd.linear.y = 0;
    velCmd.linear.z = 0;
    velCmd.angular.z = 0;

    vel_pub.publish(velCmd);

#if ST_CONSTANT_VEL
   
    ROS_INFO("Move forward ...");

    while(ros::ok())
    {
        cout << "Yaw: " << abs(rad2Deg*euler.z()) << endl;
/*
        velCmd.angular.z = 2.5 * (180*Deg2Rad - abs(euler.z()) );
        velCmd.angular.z = copysign(velCmd.angular.z, euler.z());
        cout << "yaw rate: " << velCmd.angular.z << endl;
        if( abs(velCmd.angular.z) > 1.5 )
        {
            velCmd.angular.z = copysign(1.5, velCmd.angular.z);
        }
        */
        velCmd.angular.z = 2 * (0*Deg2Rad - euler.z());
        if( abs(velCmd.angular.z) > 0.5 )
        {
            velCmd.angular.z = copysign(0.5, velCmd.angular.z);
        }
        cout << "yaw rate: " << velCmd.angular.z << endl;
/*
        if(position.x > init_pose.x + 30)
        {
            ros::Duration(0.01).sleep();
            velCmd.linear.x = -vel_desire;
            ROS_INFO("Move backward ...");
        }
        if(position.x < init_pose.x - distance)
        {
            ros::Duration(0.01).sleep();
            velCmd.linear.x = vel_desire;
            ROS_INFO("Move forward ...");
        }
*/
        velCmd.linear.x = 3; //vel_desire+2;

        vel_pub.publish(velCmd);

        ros::spinOnce();
        rate.sleep();
    }
#endif

#if ST_VARIABLE_VEL

    float acc = 0.8;
    float dt = rate.expectedCycleTime().toSec();
    float velInit = 20, vel;
    int ckey = 0, old_ckey = 0;

    cout << "dt: " << dt << endl;

    vel = velInit;
    while(ros::ok())
    {
        ckey = getch();
        if(ckey != 0)
        {
            old_ckey = ckey;
        }

        cout << "Yaw: " << rad2Deg*euler.z() << endl;
        velCmd.angular.z = 2 * (0*Deg2Rad - euler.z());
        if( abs(velCmd.angular.z) > 0.5 )
        {
            velCmd.angular.z = copysign(0.5, velCmd.angular.z);
        }
        cout << "yaw rate: " << velCmd.angular.z << endl;

        if(old_ckey != 0 && old_ckey == 104) // key (h)
        {
            vel = vel + acc * dt;
            velCmd.linear.x = vel;
        }
        else
        {
            velCmd.linear.x = velInit;
        }

        vel_pub.publish(velCmd);
        cout << "x velocity: " << vel << endl;

        ros::spinOnce();
        rate.sleep();
    }

#endif

#if ST_RANDOM_VEL
    /*********
    Thesis Data
    *******/
    float acc[6] = {0.05, -0.02, 0.05, -0.08, 0.06, -0.05}, dacc = 0.003;
    float dt = rate.expectedCycleTime().toSec();
    float velInit = 20, vel;

    cout << "dt: " << dt << endl;

    vel = velInit;

    int ckey = 0, old_ckey = 0;
    float time_st = 0, time_end = 0;
    int index = 0;
    while(ros::ok())
    {
        ckey = getch();
        if(ckey != 0)
        {
            old_ckey = ckey;
        }

        //cout << "Yaw: " << rad2Deg*euler.z() << endl;
        velCmd.angular.z = 2 * (0*Deg2Rad - euler.z());
        if( abs(velCmd.angular.z) > 0.5 )
        {
            velCmd.angular.z = copysign(0.5, velCmd.angular.z);
        }
        //cout << "yaw rate: " << velCmd.angular.z << endl;
 
        velCmd.linear.x = vel;
        vel_pub.publish(velCmd);
        cout << "x velocity: " << vel << endl;

        
        if(old_ckey != 0 && old_ckey == 104) // key (h)
        {
            vel = vel + acc[index];
            time_st += 1;
            cout << "acc index: " << index << endl;
            cout << "x acc: " << acc[index] << endl;

            if(time_st >= 200)
            {
                index++;
                time_st = 0;
                cout << "time sec: " << dt*20 << endl;

            }
            if(index >= 6)
            {
                index = 0;
            }
        }
        

        ros::spinOnce();
        rate.sleep();
    }
#endif
  
#if CIRCLE_CONSTANT_VEL

#endif

    return 0;
}