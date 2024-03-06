#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from tracking_uav_control.msg import CameraFeatures

import argparse
import numpy as np

parser = argparse.ArgumentParser(prog="python target_depth.py")
parser.add_argument("-n","--uav_num", dest="uav_num", default=3, help="uav_num")
parser.add_argument("-r","--loop_rate", dest="rate", default=100, help="loop_rate")
args = parser.parse_args()

# number of cameras
N = int(args.uav_num)

class MAV():
    def __init__(self, sub_topic, ID):
        self.pose = Point()
        self.pose_sub = rospy.Subscriber(sub_topic, Odometry, self.pose_cb, queue_size=1)
        self.id = ID

    def pose_cb(self, msg):
        self.pose = msg.pose.pose.position

    def getPose(self):
        return self.pose
    
class DNN():
    def __init__(self, sub_topic, ID):
        self.bbx = CameraFeatures()
        self.detect_sub = rospy.Subscriber(sub_topic, CameraFeatures, self.detect_cb, queue_size=1)

    def detect_cb(self, msg):
        self.bbx = msg

    def getBBX(self):
        return self.bbx

def DataCallback():
    global pose_uav, info_cam
    pose_uav = []
    info_cam = []
    for i in range(N):
        pose_uav.append( MAV("/uav"+str(i)+"/base_pose_ground_truth", i) )
        info_cam.append( DNN("/uav"+str(i)+"/estimation/ukf/camera_features", i) )
cnt=0

class DepthEstimate():
    def __init__(self):
        rospy.init_node('target_depth', anonymous=True)
        self.rate = rospy.Rate(int(args.rate))
        
        DataCallback()

    def estimate(self):
        tau = []
        dummy = np.zeros([3,1])
        for i in range(N):
            tau.append( np.matrix([
                    [info_cam[i].getBBX().fx.data],
                    [-info_cam[i].getBBX().u.data],
                    [-info_cam[i].getBBX().v.data] ]) )
            print(tau[i])
            
        
        A = np.vstack( ( np.hstack((tau[0],-tau[1],dummy)), np.hstack((tau[0],dummy,-tau[2])), np.hstack((dummy,tau[1],-tau[2])) ) )
        print('A ', A)

        b = []
        for i in range(N-1):
            for j in range(i+1, N):
                b.append( np.matrix([
                    [pose_uav[i].getPose().x - pose_uav[j].getPose().x],
                    [pose_uav[i].getPose().y - pose_uav[j].getPose().y],
                    [pose_uav[i].getPose().z - pose_uav[j].getPose().z] ]) )
                
        B = np.vstack((b[0], b[1], b[2]))
        print('B ',B)

        global cnt
        cnt = cnt+1
        
        if cnt > 10:
            ATA = np.dot( np.transpose(A), A )
            inv_ATA = np.linalg.inv(ATA)
            pinv_A = np.dot( inv_ATA,  np.transpose(A))
            X = np.dot( pinv_A, B)
            print("X ", X)

            p1 = b[0]+X[0,0]*tau[0]
            p2 = b[1]+X[1,0]*tau[1]
            p3 = b[2]+X[2,0]*tau[2]

            d1 = np.sqrt( p1[0]**2+p1[1]**2+p1[2]**2 )
            d2 = np.sqrt( p2[0]**2+p2[1]**2+p2[2]**2 )
            d3 = np.sqrt( p3[0]**2+p3[1]**2+p3[2]**2 )

            print('d1 ', d1)
            print('d2 ', d2)
            print('d3 ', d3)



if __name__ == '__main__':
    dep = DepthEstimate()

    while not rospy.is_shutdown():
        dep.estimate()
