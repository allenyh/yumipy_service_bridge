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
        self.timer = rospy.Timer(rospy.Duration(0.003), self.logCallBack)
        rospy.Service("/get_pose", GetPose, self.get_pose_cb)

    def logCallBack(self, event):
        msgs = self.left_arm_socket.recv(100)
        msgs_list = self.process(msgs)
        if len(msgs_list) == 0:
            return
        trans = [msgs_list[1]/1000, msgs_list[2]/1000, msgs_list[3]/1000] # unit: m
        quat = msgs_list[4:]
        quat = [quat[1], quat[2], quat[3], quat[0]] # wxyz -> xyzw
        self.left_arm_pose = trans + quat

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
        res.pose = self.left_arm_pose
        return res


if __name__ == '__main__':
    rospy.init_node('JointLogger')
    node = JointLogger()
    rospy.spin()