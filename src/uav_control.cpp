#include "FwControl.hpp"

#define LOITER_WAYPOINT 0
#define LOITER 0
#define STRAIGHT 1
#define UPNDOWN 0

int main(int argc, char **argv)
{
    ros::init(argc, argv, "uav_control");
    FwControl fw;
    bool state;
     
    state = fw.CheckFCUConnection();
    std::cout << "FCU Connection State: " << state << std::endl;
    state = fw.Initialize();
    std::cout << "Finished Initializing: " << state << std::endl;
    fw.SwitchFlightMode("OFFBOARD");
    //fw.SwitchMode("OFFBOARD");
    std::cout << "Switch to Offboard mode. Ready to fly!" << std::endl;
    //fw.TakeoffVel();


#if LOITER_WAYPOINT
    std::vector<float> targetCtr ({0, 0, 20});
    std::cout << "Start Loitering" << std::endl;
    fw.TrackCircle(targetCtr, 60, 0.25/(2*M_PI));
#endif    

#if LOITER
    fw.Loiter();
#endif

#if STRAIGHT

    fw.TakeoffPos();
    fw.GoStraight();

    //fw.TakeoffVel();
    //fw.GoStraightVel();

#endif

#if UPNDOWN

    fw.StraightUpNDown();

#endif

    return 0;
}