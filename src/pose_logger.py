#!/usr/bin/env python

import rospy
import socket
from yumipy.yumi_constants import YuMiConstants as YMC
from yumipy_service_bridge.srv import GetPose, GetPoseResponse


class JointLogger:
    def __init__(self):
        self.ip = YMC.IP
        self.left_arm_port = YMC.PORTS['left']["poses"]
        self.left_arm_pose = list()
        self.left_arm_socket = socket.socket()
        self.left_arm_socket.connect((self.ip, self.left_arm_port))
        self.right_arm_port = YMC.PORTS['right']["poses"]
        self.right_arm_pose = list()
        self.right_arm_socket = socket.socket()
        self.right_arm_socket.connect((self.ip, self.right_arm_port))
        self.timer = rospy.Timer(rospy.Duration(0.003), self.logCallBack)
        rospy.Service("/get_pose", GetPose, self.get_pose_cb)


    def logCallBack(self, event):
        msgs_list = self.process(self.left_arm_socket.recv(100))
        if len(msgs_list) == 0:
            return
        trans = [float(msgs_list[1])/1000, float(msgs_list[2])/1000, float(msgs_list[3])/1000] # unit: m
        quat = msgs_list[4:]
        quat = [float(quat[1]), float(quat[2]), float(quat[3]), float(quat[0])] # wxyz -> xyzw
        self.left_arm_pose = trans + quat

        msgs_list = self.process(self.right_arm_socket.recv(100))
        if len(msgs_list) == 0:
            return
        trans = [float(msgs_list[1]) / 1000, float(msgs_list[2]) / 1000, float(msgs_list[3]) / 1000]  # unit: m
        quat = msgs_list[4:]
        quat = [float(quat[1]), float(quat[2]), float(quat[3]), float(quat[0])]  # wxyz -> xyzw
        self.right_arm_pose = trans + quat

    def process(self, msgs):
        index_pound = msgs.find('#')
        if index_pound == -1:
            return list()
        msgs = msgs[index_pound + 1:]
        index_exclam = msgs.find('!')
        if index_exclam == -1:
            return list()
        msgs = msgs[:index_exclam]
        msgs_list = msgs.split()
        if len(msgs_list) != 8:
            return list()
        return msgs_list

    def get_pose_cb(self, req):
        res = GetPoseResponse()
        if req.arm == 'left':
            res.pose = self.left_arm_pose
        elif req.arm == 'right':
            res.pose = self.right_arm_pose
        return res


if __name__ == '__main__':
    rospy.init_node('PoseLogger')
    node = JointLogger()
    rospy.spin()