#!/usr/bin/env python

import rospy
import socket
from sensor_msgs.msg import JointState
from yumipy.yumi_constants import YuMiConstants as YMC


class JointLogger:
    def __init__(self):
        self.joint_log_pub = rospy.Publisher('/joint_states', JointState, queue_size=1)
        self.ip = YMC.IP
        self.left_arm_port = YMC.PORTS['left']["states"]
        self.right_arm_port = YMC.PORTS['right']['states']
        self.left_arm_name = ["yumi_joint_1_l", "yumi_joint_2_l", "yumi_joint_3_l",
                              "yumi_joint_4_l", "yumi_joint_5_l", "yumi_joint_6_l", "yumi_joint_7_l"]
        self.right_arm_name = ["yumi_joint_1_r", "yumi_joint_2_r", "yumi_joint_3_r",
                               "yumi_joint_4_r", "yumi_joint_5_r", "yumi_joint_6_r", "yumi_joint_7_r"]
        self.left_arm_socket = socket.socket()
        self.right_arm_socket = socket.socket()
        self.left_arm_socket.connect((self.ip, self.left_arm_port))
        self.right_arm_socket.connect((self.ip, self.right_arm_port))
        self.timer = rospy.Timer(rospy.Duration(0.003), self.logCallBack)
        self.joints = list()

    def logCallBack(self, event):
        js = JointState()
        js.header.frame_id = 'map'
        js.header.stamp = rospy.Time.now()
        js.name = self.left_arm_name
        msgs = self.left_arm_socket.recv(100)
        msgs_list = self.process(msgs)
        if len(msgs_list) == 0:
            return
        self.joints = list()
        for i in range(len(msgs_list)):
            #if i == 0:
            #    self.joints.append(float(msgs_list[i])/10000) # convert to m, the value is 250 when the width is 25mm
            #else:
            self.joints.append(float(msgs_list[i]) * 0.017453292)  # convert to rad
        js.position = self.joints
        self.joint_log_pub.publish(js)

        js.header.stamp = rospy.Time.now()
        js.name = self.right_arm_name
        msgs = self.right_arm_socket.recv(100)
        msgs_list = self.process(msgs)
        if len(msgs_list) == 0:
            return
        self.joints = list()
        for i in range(len(msgs_list)):
            #if i == 0:
            #    self.joints.append(float(msgs_list[i])/10000) # convert to m, the value is 250 when the width is 25mm
            #else:
            self.joints.append(float(msgs_list[i]) * 0.017453292)  # convert to rad
        js.position = self.joints
        self.joint_log_pub.publish(js)

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
        if len(msgs_list) != 7:
            return list()
        return msgs_list


if __name__ == '__main__':
    rospy.init_node('JointLogger')
    node = JointLogger()
    rospy.spin()