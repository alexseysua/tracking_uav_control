#include <iostream>
#include <cmath>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include "util.cpp"

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "camera_zoom_control");
    ros::NodeHandle nh;
    ros::Rate rate(20);
    ros::Publisher zoom_pub = nh.advertise<std_msgs::Float64>("/uav0/camera_ir/set_zoom", 2);

    std_msgs::Float64 set_zoom;
    float hov_1x = 0.54, fl_1x = 2283.4293;
    int zoom_scale = 1;
    float hov, fl;

    while(ros::ok())
    {
        
        set_zoom.data = zoom_scale;
        fl = set_zoom.data * fl_1x;
        hov = 2 * atan2( 1280, 2*fl );
        cout << "Camera zoom: " << zoom_scale << "x" << endl << "HOV: " << hov << endl << endl;
        
        //ros::Duration(0.2).sleep();

        zoom_pub.publish(set_zoom);
        
        zoom_scale++;
        if(zoom_scale > 30)
        {
            break;
        }

        ros::spinOnce();
        rate.sleep();
    }
  
    return 0;
}