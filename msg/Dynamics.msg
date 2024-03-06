##------ under camera frame ------##
geometry_msgs/Vector3 target_vel
geometry_msgs/Vector3 uav_vel
geometry_msgs/Vector3 uav_wvel
geometry_msgs/Vector3 pt_wvel

geometry_msgs/Vector3 camuav_pose
geometry_msgs/Vector3 targetcam_pose
std_msgs/Float64 depth

##------ under inertial frame ------##
geometry_msgs/Vector3 target_vel_inertial
geometry_msgs/Vector3 cam_pose_inertial
geometry_msgs/Vector3 target_pose_inertial

std_msgs/Float32MultiArray rot_g2c
geometry_msgs/Vector3 trans_g2c
std_msgs/Float32MultiArray rot_c2g
geometry_msgs/Vector3 trans_c2g