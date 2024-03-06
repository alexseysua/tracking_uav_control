#include "GimbalControl.hpp"

#define TRACK 0
#define TRACK_REVISED 1
#define GIMBAL_SHOW1 0
#define GIMBAL_SHOW2 0
#define LOCK 0

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gimbal_control");
    GimbalControl gb;  

#if TRACK
    std::cout << "start tracking ......" << std::endl;
    gb.TrackingController(gb.ALWAYSON, gb.typhoon, gb.measure_off);
#endif

#if TRACK_REVISED
    std::cout << "start tracking ......" << std::endl;
    gb.TrackingControllerR(gb.ALWAYSON, gb.miniyy, gb.measure_off);   //gb.SEARCH, gb.ALWAYSON, gb.HOLD
#endif

#if GIMBAL_SHOW1
    std::cout << "Start glmbal control." << std::endl;
    gb.MountShow1();
    std::cout << "End...." << std::endl;S
#endif

#if GIMBAL_SHOW2
    std::cout << "Start glmbal control." << std::endl;
    gb.MountShow2();
    std::cout << "End...." << std::endl;
#endif

#if LOCK
    std::cout << "Angle Lock." << std::endl;
    gb.AngleLock();
    std::cout << "End...." << std::endl;
#endif


    return 0;
}