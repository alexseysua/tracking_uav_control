#include "FwControl.hpp"
#include "util.cpp"

FwControl::FwControl()
{
    state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, &FwControl::getCurrentState, this);
    global_pos_sub = nh.subscribe<sensor_msgs::NavSatFix>
            ("mavros/global_position/global", 10, &FwControl::getGlobalLLA, this);
    hud_sub = nh.subscribe<mavros_msgs::VFR_HUD>
            ("/uav0/mavros/vfr_hud", 10, &FwControl::hud_cb, this);
    fw_pose_sub = nh.subscribe<nav_msgs::Odometry>
            ("base_pose_ground_truth", 10, &FwControl::getFwPose, this);
    vel_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("mavros/setpoint_velocity/cmd_vel", 10);
    localPos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    att_pub = nh.advertise<mavros_msgs::AttitudeTarget>
            ("/uav0/mavros/setpoint_raw/attitude", 10);
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    takeoff_client = nh.serviceClient<mavros_msgs::CommandTOL>
            ("mavros/cmd/takeoff");
}

FwControl::~FwControl()
{

}

void FwControl::getCurrentState(const mavros_msgs::State::ConstPtr& state)
{
    current_state = *state;
}

void FwControl::getGlobalLLA(const sensor_msgs::NavSatFix::ConstPtr& lla)
{
    current_lla = *lla;
}

void FwControl::hud_cb(const mavros_msgs::VFR_HUD::ConstPtr& msg)
{
    current_hud = *msg;
}

void FwControl::getFwPose(const nav_msgs::Odometry::ConstPtr& pose)
{
    quat_local = Eigen::Quaternionf( pose->pose.pose.orientation.w, pose->pose.pose.orientation.x, pose->pose.pose.orientation.y, pose->pose.pose.orientation.z );
    euler_local = Quaternion2Euler(quat_local);
}

bool FwControl::CheckFCUConnection()
{
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    return true;
}

bool FwControl::Initialize()
{
    cmd_pos.pose.position.x = 10.0;
    cmd_pos.pose.position.y = 0.0;
    cmd_pos.pose.position.z = 30.0;
    for (int i = 60; ros::ok() && i > 0; i--){
        localPos_pub.publish(cmd_pos);
        //ros::spinOnce();
        rate.sleep();
    }

    return true;
}

void FwControl::SwitchMode(std::string flightMode)
{
    offb_set_mode.request.custom_mode = flightMode;
    cmd_arm.request.value = true;

    cmd_takeoff.request.latitude = current_lla.latitude;
    cmd_takeoff.request.longitude = current_lla.longitude + 0.004;
    cmd_takeoff.request.altitude = current_lla.altitude + 100.0;

    last_request = ros::Time::now();
    while(ros::ok()){
        if( !current_state.armed &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( arming_client.call(cmd_arm) &&
                cmd_arm.response.success){
                ROS_INFO("Vehicle armed");
            }
            last_request = ros::Time::now();
        } else if( !isTakeoff && current_state.armed){
                takeoff_client.call(cmd_takeoff);
                ROS_INFO("Takeoff");
                isTakeoff = cmd_takeoff.response.success;
        } else {
            if( current_state.mode != flightMode && isTakeoff &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( set_mode_client.call(offb_set_mode) &&
                    offb_set_mode.response.mode_sent ){
                    ROS_INFO("Offboard enabled");
                }
                last_request = ros::Time::now();
            }
        }
        
        if(current_state.armed && isTakeoff && current_state.mode == flightMode){
            break;
        }

        localPos_pub.publish(cmd_pos);
        ros::spinOnce();
        rate.sleep();
    }
}

void FwControl::SwitchFlightMode(std::string flightMode)
{
    offb_set_mode.request.custom_mode = flightMode;
    cmd_arm.request.value = true;

    last_request = ros::Time::now();
    while(ros::ok())
    {
        if( !current_state.armed &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( arming_client.call(cmd_arm) &&
                cmd_arm.response.success){
                ROS_INFO("Vehicle armed");
            }
            last_request = ros::Time::now();
        }
        else 
        {
            if ( current_state.mode != flightMode &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent ){
                ROS_INFO("%s enabled", flightMode.c_str());
            }
            last_request = ros::Time::now();
            }
        }   
        if(current_state.mode == flightMode){
            break;
        }

        cmd_vel.twist.linear.x = 5;
        cmd_vel.twist.linear.y = 0;
        cmd_vel.twist.linear.z = 3;

        vel_pub.publish(cmd_vel);

        ros::spinOnce();
        rate.sleep();
    }
}

void FwControl::Takeoff()
{
    cmd_takeoff.request.latitude = current_lla.latitude;
    cmd_takeoff.request.longitude = current_lla.longitude + 0.008;
    cmd_takeoff.request.altitude = current_lla.altitude + 100;
    
    while(ros::ok()){
        if(!isTakeoff){
            if( current_state.mode == controlMode && current_state.armed)
            {
                takeoff_client.call(cmd_takeoff);
                ROS_INFO("Takeoff");
                isTakeoff = cmd_takeoff.response.success;
            }
        }

        vel_pub.publish(cmd_vel);
        ros::spinOnce();
        rate.sleep();
    }
}

void FwControl::TakeoffPos()
{
    int counter = 0;
    ROS_INFO("Takeoff");
    while(ros::ok())
    {

        cmd_att.thrust = 0.7; 
        att_pub.publish(cmd_att);
        counter++;
        if(counter > 5)
        {
            break;
        }
        rate.sleep();
    }
}

void FwControl::TakeoffVel()
{
    int counter = 0;
    double intAltitue = 488, targetAltitue = 1500+488;
    Eigen::Vector3f euler_desire;
    Eigen::Quaternionf q_desire;
    ROS_INFO("Takeoff");

    while(ros::ok())
    {
        euler_desire = euler_local;
        euler_desire[2] = 0;

        q_desire = Quaternion2Euler(euler_desire);

        cout << "euler: " << endl << euler_local << endl;
        cout << "q: " << quat_local.x() << ", " << quat_local.y() << ", " << quat_local.z() << ", " << quat_local.w() << endl;
        cout << "q after: " << q_desire.x() << ", " << q_desire.y() << ", " << q_desire.z() << ", " << q_desire.w() << endl << endl;

        cout << current_hud.altitude << endl;
        /*
        if(current_hud.altitude < 510)
        {
            cmd_att.type_mask = 0;


            cmd_att.thrust = 0.6;
            cmd_att.body_rate.z = 0;
            
        }
        else
        {*/
            //cmd_att.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;
            cmd_att.thrust = 0.7;
            //cmd_att.body_rate.x = 0.001 * (0 - euler_local[0]);
            //cmd_att.body_rate.z = - 0.01 * (90 - current_hud.heading); 
            //cout << "yaw rate: " << cmd_att.body_rate.z << endl;
        //}
        
        cout << endl;

        if(current_hud.altitude > 630)
        {
            break;
        }
        
        att_pub.publish(cmd_att);
        counter++;
        ros::spinOnce();
        rate.sleep();
    }
}    

void FwControl::GoStraightVel()
{
    float dist = 1000, veldes = 33;
    ROS_INFO("Go Straight ...");
    int counter = 0;
    bool poss = true;

    while(ros::ok())
    {
        cmd_att.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;
        cmd_att.thrust = 0.4 + 0.08 * (veldes - current_hud.groundspeed); 
        
        cmd_att.body_rate.x = 0.00003;

        att_pub.publish(cmd_att);
        counter++;
        rate.sleep();
    }
}

void FwControl::GoStraight()
{
    float dist = 1000;
    int counter = 0;
    ROS_INFO("Go Straight ...");

    while(ros::ok())
    {
        cmd_pos.pose.position.x = dist;
        cmd_pos.pose.position.y = 0;
        cmd_pos.pose.position.z = 50;
        dist += 20;

        localPos_pub.publish(cmd_pos);
        rate.sleep();
    }
}

void FwControl::StraightUpNDown()
{
    float dist = 500;
    int counter = 0;

    while(ros::ok())
    {
        ROS_INFO("Go Straight ...");
        cmd_pos.pose.position.x = dist;
        cmd_pos.pose.position.y = 0;

        if(counter < 300)
        {
            cmd_pos.pose.position.z = 600;
        }
        if(counter >= 300)
        {
            cmd_pos.pose.position.z = 100;
        }
        if(counter >= 400)
        {
            cmd_pos.pose.position.z = 600;
        }

        dist += 10;

        localPos_pub.publish(cmd_pos);
        counter++;
        rate.sleep();
    }
}

void FwControl::Loiter()
{
    float dist = 100;
    int counter = 0;

    while(ros::ok())
    {
        if(dist < 200)
        {
            ROS_INFO("Go Straight ...");
            cmd_pos.pose.position.x = dist;
            cmd_pos.pose.position.y = 0;
            cmd_pos.pose.position.z = 30;
            dist += 5;

            localPos_pub.publish(cmd_pos);
            rate.sleep();
        }
        if(dist >= 200 && counter < 4)
        {
            ROS_INFO("Sending one fixed position ...");
            cmd_pos.pose.position.x = 0;
            cmd_pos.pose.position.y = 0;
            cmd_pos.pose.position.z = 30;
            counter++;

            localPos_pub.publish(cmd_pos);
            rate.sleep();
        }

        FwControl::SwitchFlightMode("AUTO.LOITER");
    }
}

void FwControl::TrackCircle(std::vector<float> ctr, float radius, float freq)
{
    float period = 200; //second
    float step = 0.01;
    float omega = 2*M_PI*freq;
    geometry_msgs::Point waypoint;
    geometry_msgs::Point errPos;

    std::vector<std::vector<float>> guidePath ({{50, 0}, {55, 10}, {60, 20}, {60, 30}, {20, 20}});

    for(int i = 0; i < 5; i++){
        cmd_pos.pose.position.x = guidePath[i][0];
        cmd_pos.pose.position.y = guidePath[i][0];
        cmd_pos.pose.position.z = ctr[2];

        std::cout << "Guide the location: " << cmd_pos.pose.position.x \
            << ", " << cmd_pos.pose.position.y << ", " << cmd_pos.pose.position.z \
            << std::endl;
    }


    for(float t = 0; t <= period;){
        waypoint.x = ctr[0] + radius*sin(omega*t);
        waypoint.y = ctr[1] + radius*cos(omega*t);
        waypoint.z = ctr[2];

        cmd_pos.pose.position.x = waypoint.x;
        cmd_pos.pose.position.y = waypoint.y;
        cmd_pos.pose.position.z = waypoint.z;

        std::cout << "Pass through the local location: " << cmd_pos.pose.position.x \
            << ", " << cmd_pos.pose.position.y << ", " << cmd_pos.pose.position.z \
            << std::endl;

        localPos_pub.publish(cmd_pos);
        rate.sleep();
        
        t += step;
    }
}