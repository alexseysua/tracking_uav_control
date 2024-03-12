#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from std_msgs.msg import Int16
from geometry_msgs.msg import Vector3
from tracking_uav_control.msg import CameraFeatures
from tracking_uav_control.msg import LSEDepth

import argparse
import numpy as np
import math as m
from tf.transformations import quaternion_matrix
from utils import *

parser = argparse.ArgumentParser(prog="python target_depth.py")
parser.add_argument("-n","--uav_num", dest="uav_num", default=3, help="uav_num")
parser.add_argument("-r","--loop_rate", dest="rate", default=100, help="loop_rate")
args = parser.parse_args()

# number of cameras
N = int(args.uav_num)

class MAV():
    def __init__(self, sub_topic, ID):
        self.position = Point()
        self.orientation = Quaternion()
        self.pose_sub = rospy.Subscriber(sub_topic, Odometry, self.pose_cb, queue_size=1)
        self.id = ID

    def pose_cb(self, msg):
        self.position = msg.pose.pose.position
        self.orientation = msg.pose.pose.orientation

    def getPos(self):
        return self.position
    
    def getQua(self):
        return self.orientation
    
    def getID(self):
        return self.id
    
class DNN():
    def __init__(self, sub_topic, ID):
        super().__init__()
        self.bbx = CameraFeatures()
        self.detect_sub = rospy.Subscriber(sub_topic, CameraFeatures, self.detect_cb, queue_size=1)
        self.vec = np.zeros([3,1])
        self.feature = {'u':0, 'v':0, 'f':0, 'cu':0, 'cv':0}
        self.__feature = {'u':0, 'v':0, 'f':0, 'cu':0, 'cv':0}
        self.id = ID

    def detect_cb(self, msg):
        self.bbx = msg
        self.__feature['u'] = self.bbx.u.data
        self.__feature['v'] = self.bbx.v.data
        self.__feature['f'] = self.bbx.fx.data
        self.__feature['cu'] = self.bbx.cu.data
        self.__feature['cv'] = self.bbx.cv.data
        if abs(self.__feature['u']) >= 0.5 and abs(self.__feature['v']) >= 0.5 and abs(self.__feature['f']) >= 0.5 and abs(self.__feature['cu']) >= 0.5 and abs(self.__feature['cv']) >= 0.5:
            self.feature['u'] = self.__feature['u']
            self.feature['v'] = self.__feature['v']
            self.feature['f'] = self.__feature['f']
            self.feature['cu'] = self.__feature['cu']
            self.feature['cv'] = self.__feature['cv']
        self.vec[0] = (self.feature['u'] - self.bbx.cu.data)
        self.vec[1] = (self.feature['v'] - self.bbx.cv.data)
        self.vec[2] =  self.feature['f']

    def getBBX(self):
        return self.bbox
    
    def getVEC(self):
        return self.vec
    
    def getData(self):
        return abs(self.__feature['u']) >= 0.5 and abs(self.__feature['v']) >= 0.5 and abs(self.__feature['f']) >= 0.5 and abs(self.__feature['cu']) >= 0.5 and abs(self.__feature['cv']) >= 0.5
    
class GIMBAL():
    def __init__(self, sub_topic, ID):
        self.__states = JointState()
        self.pose_sub = rospy.Subscriber(sub_topic, Odometry, self.pose_cb, queue_size=1)
        self.state = {'roll': 0,
                      'tilt': 0,
                      'pan':  0,
                      'roll_rate': 0, 
                      'tilt_rate': 0,
                      'pan_rate':  0}
        self.id = ID

    def pose_cb(self, msg):
        self.__states = msg
        self.state['roll'] = self.__states.position[0]
        self.state['tilt'] = self.__states.position[1]
        self.state['pan'] = self.__states.position[2]
        self.state['roll_rate'] = self.__states.velocity[0]
        self.state['tilt_rate'] = self.__states.velocity[1]
        self.state['pan_rate'] = self.__states.velocity[2]

    def getPos(self):
        return self.state

def DataCallback():
    global pose_uav, info_cam, pose_gimbal
    pose_uav = []
    info_cam = []
    pose_gimbal = []

    for i in range(N):
        pose_uav.append( MAV("/uav"+str(i)+"/base_pose_ground_truth", i) )
        info_cam.append( DNN("/uav"+str(i)+"/estimation/ukf/camera_features", i) )
        pose_gimbal.append( GIMBAL("/uav"+str(i)+"/gimbal/joint_states", i) )

def LoadingGeometry():
    global geo_uav2pan, geo_pan2cam
    geo_uav2pan = [] 
    geo_pan2cam = [] 
    for i in range(N):
        # flu frame
        geo_uav2pan.append(np.array([[rospy.get_param('/uav'+str(i)+'/miniyy/relPos_panPlane/x')],
                                [rospy.get_param('/uav'+str(i)+'/miniyy/relPos_panPlane/y')],
                                [rospy.get_param('/uav'+str(i)+'/miniyy/relPos_panPlane/z')]]))
        # pan frame
        geo_pan2cam.append(np.array([[rospy.get_param('/uav'+str(i)+'/miniyy/relPos_camPan/x')],
                                [rospy.get_param('/uav'+str(i)+'/miniyy/relPos_camPan/y')],
                                [rospy.get_param('/uav'+str(i)+'/miniyy/relPos_camPan/z')]]))

def GimbalGeometryTransfer():
    global geo_gimbal
    geo_gimbal = []
    for i in range(N):
        # convert to pan frame
        R_l2p = np.dot( R_b2p(pose_gimbal[i].getPos()['pan']), R_l2b() )
        temp = np.dot( R_l2p, geo_uav2pan[i] ) + geo_pan2cam[i]
        # convert to camera frame
        temp = np.dot( R_p2c(pose_gimbal[i].getPos()['tilt']), temp )
        # camera frame convert to gnd frame
        geo_gimbal.append( np.dot( invR_g2c(pose_gimbal[i].getPos()['tilt'], pose_gimbal[i].getPos()['pan'], pose_uav[i].getQua()), temp))

class DepthEstimate():
    def __init__(self):
        rospy.init_node('target_depth', anonymous=True)
        self.pub_depth = rospy.Publisher('/estimate/lse/depth', LSEDepth, queue_size=1)
        self.rate = rospy.Rate(int(args.rate))
        self.results = LSEDepth()
        
        DataCallback()
        LoadingGeometry()

    def estimate(self):
        # transfer (UAV->camera) geometry into gnd frame
        GimbalGeometryTransfer()

        # camera position in gnd frame
        pose_cam = []
        for i in range(N):
            pose_cam.append( np.array([
                [pose_uav[i].getPos().x],
                [pose_uav[i].getPos().y],
                [pose_uav[i].getPos().z] ]) + geo_gimbal[i])

        if all(info_cam[i].getData() for i in range(N)):

            for i in range(N):
                print(info_cam[i].getVEC())


            # construct matrices
            tau = []
            dummy = np.zeros([3,1])
            for i in range(N):
                vec = np.dot( invR_g2c(pose_gimbal[i].getPos()['tilt'], pose_gimbal[i].getPos()['pan'], pose_uav[i].getQua()), info_cam[i].getVEC() )
                tau.append(vec)
                    
            A = np.vstack( ( np.hstack((tau[0],-tau[1],dummy)), np.hstack((tau[0],dummy,-tau[2])), np.hstack((dummy,tau[1],-tau[2])) ) )
            b = []
            for i in range(N-1):
                for j in range(i+1, N):
                    b.append( pose_cam[j] - pose_cam[i] )
                    
            B = np.vstack((b[0], b[1], b[2]))
            # LSE
            ATA = np.dot( np.transpose(A), A )
            inv_ATA = np.linalg.inv(ATA)
            pinv_A = np.dot( inv_ATA,  np.transpose(A))
            X = np.dot( pinv_A, B)

            pt=[]
            for i in range(N):
                pos = Vector3()
                _pos = pose_cam[i]+X[i,0]*tau[i]
                pos.x = _pos[0,0]
                pos.y = _pos[1,0]
                pos.z = _pos[2,0]
                pt.append(pos)

            di = []
            for i in range(N):
                _di = X[i,0]*tau[i]
                di.append( np.sqrt( (_di[0])**2+(_di[1])**2+(_di[2])**2 ) )

            self.results.mav_id = [Int16(pose_uav[0].getID()), Int16(pose_uav[1].getID()), Int16(pose_uav[2].getID())]
            self.results.depth = [Float64(di[0]), Float64(di[1]), Float64(di[2])]
            self.results.position = pt
            self.pub_depth.publish(self.results)


if __name__ == '__main__':
    dep = DepthEstimate()

    while not rospy.is_shutdown():
        dep.estimate()
        
