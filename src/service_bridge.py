#!/usr/bin/env python
import rospy
from yumipy import YuMiRobot, YuMiMotionPlanner
from yumipy.yumi_util import message_to_pose, message_to_state
from yumipy_service_bridge.srv import Trigger, TriggerResponse, GetPose, GetPoseResponse, \
    GotoPose, GotoPoseResponse, GotoJoint, GotoJointResponse, MoveGripper, MoveGripperResponse, \
    SetZ, SetZResponse, GotoJointRequest


class ServiceBridge:
    def __init__(self):
        #self.left_arm_planner = YuMiMotionPlanner(arm='left')
        #self.right_arm_planner = YuMiMotionPlanner(arm='right')
        self.robot = YuMiRobot(arm_type='remote')
        self.left_arm = self.robot.left
        self.right_arm = self.robot.right
        #self.left_arm.set_motion_planner()
        #self.right_arm.set_motion_planner()
        self._create_services()
        rospy.loginfo("[ServiceBridge] Node is up!")

    def _create_services(self):
        rospy.Service("/goto_pose", GotoPose, self.goto_pose_cb)
        rospy.Service("/goto_joints", GotoJoint, self.goto_joints_cb)
        rospy.Service("/close_gripper", Trigger, self.close_gripper_cb)
        rospy.Service("/open_gripper", Trigger, self.open_gripper_cb)
        rospy.Service("/move_gripper", MoveGripper, self.move_gripper_cb)
        rospy.Service("/get_gripper_width", Trigger, self.get_gripper_width)
        #rospy.Service("/set_speed", , self.set_speed_cb)
        rospy.Service("/set_zone", SetZ, self.set_zone_cb)
        rospy.Service("/reset_home", Trigger, self.reset_home_cb)
        rospy.Service("/calibrate_gripper", Trigger, self.calibrate_gripper_cb)
        rospy.Service("/turn_on_suction", Trigger, self.turn_on_suction_cb)
        rospy.Service("/turn_off_suction", Trigger, self.turn_off_suction_cb)
        rospy.Service("goto_wait_joint", Trigger, self.goto_wait_joint_cb)
        rospy.Service("goto_scan_joint", Trigger, self.goto_scan_joint_cb)

    def goto_pose_cb(self, req):
        response = GotoPoseResponse()
        response.success = False
        if (req.arm != 'left' and req.arm != 'right') or len(req.position) != 3 or len(req.quat) != 4:
            return response
        pose = ''
        for p in req.position:
            pose += str(p) + ' '
        for q in req.quat:
            pose += str(q) + ' '
        pose = message_to_pose(pose, 'tool')
        if req.arm == 'left':
            #if self.left_arm.is_pose_reachable(pose):
            self.left_arm.goto_pose_shortest_path(pose, wait_for_res=req.wait_for_res)
        else:
            #if self.right_arm.is_pose_reachable(pose):
            print("wait for res: {}".format(req.wait_for_res))
            self.right_arm.goto_pose_shortest_path(pose, wait_for_res=req.wait_for_res)
        response.success = True
        return response

    def goto_joints_cb(self, req):
        response = GotoJointResponse()
        response.success = False
        if (req.arm != 'left' and req.arm != 'right') or len(req.joint) != 7:
            return response
        joints = ''
        for j in req.joint:
            joints += str(j) + ' '
        state = message_to_state(joints)
        if req.arm == 'left':
            self.left_arm.goto_state(state)
        else:
            self.right_arm.goto_state(state)
        response.success = True
        return response

    def close_gripper_cb(self, req):
        response = TriggerResponse()
        response.success = True
        if req.arm == 'left':
            self.left_arm.close_gripper()
        elif req.arm == 'right':
            self.right_arm.close_gripper()
        else:
            rospy.logerr("[Close Gripper] No arm named {}".format(req.arm))
            response.success = False
            response.message = "No arm named {}".format(req.arm)
            return response
        response.message = 'Successfully close {} gripper'.format(req.arm)
        return response

    def open_gripper_cb(self, req):
        response = TriggerResponse()
        response.success = True
        if req.arm == 'left':
            self.left_arm.open_gripper()
        elif req.arm == 'right':
            self.right_arm.open_gripper()
        else:
            rospy.logerr("[Open Gripper] No arm named {}".format(req.arm))
            response.success = False
            response.message = "No arm named {}".format(req.arm)
            return response
        response.message = 'Successfully open {} gripper'.format(req.arm)
        return response

    def move_gripper_cb(self, req):
        response = MoveGripperResponse()
        response.success = False
        arm = None
        if req.arm == 'left':
            arm = self.left_arm
        elif req.arm == 'right':
            arm = self.right_arm
        if arm is not None:
            if req.width < 0:
                arm.close_gripper()
            elif req.width >= 0.025:
                arm.open_gripper()
            else:
                arm.move_gripper(req.width)
            response.success = True

        return response

    # def set_speed_cb(self, req):
    #     # TODO write
    #
    def set_zone_cb(self, req):
        zone_value = req.z
        self.robot.set_z(zone_value)

        return SetZResponse()

    def get_gripper_width(self, req):
        response = TriggerResponse()
        response.success = True
        if req.arm == 'left':
            width = self.left_arm.get_gripper_width()
        elif req.arm == 'right':
            width = self.right_arm.get_gripper_width()
        else:
            rospy.logerr("[Get Gripper Width] No arm named {}".format(req.arm))
            response.success = False
            response.message = "No arm named {}".format(req.arm)
            return response
        response.message = str(width)
        return response

    def reset_home_cb(self, req):
        response = TriggerResponse()
        response.success = True
        if req.arm == 'left':
            self.left_arm.reset_home()
        elif req.arm == 'right':
            self.right_arm.reset_home()
        else:
            rospy.logerr("[Reset Home] No arm named {}".format(req.arm))
            response.success = False
            response.message = "No arm named {}".format(req.arm)
            return response
        response.message = 'Successfully reset {} arm to home'.format(req.arm)
        return response

    def calibrate_gripper_cb(self, req):
        response = TriggerResponse()
        response.success = True
        if req.arm == 'left':
            self.left_arm.calibrate_gripper()
        elif req.arm == 'right':
            self.right_arm.calibrate_gripper()
        else:
            rospy.logerr("[Reset Home] No arm named {}".format(req.arm))
            response.success = False
            response.message = "No arm named {}".format(req.arm)
            return response
        response.message = 'Successfully calibrate {} gripper'.format(req.arm)
        return response

    def turn_on_suction_cb(self, req):
        response = TriggerResponse()
        response.success = True
        if req.arm == 'left':
            self.left_arm.turn_on_suction()
        elif req.arm == 'right':
            self.right_arm.turn_on_suction()
        else:
            rospy.logerr("[Turn On Suction] No arm named {}".format(req.arm))
            response.success = False
            response.message = "No arm named {}".format(req.arm)
            return response
        response.message = 'Successfully Turn on {} suction'.format(req.arm)
        return response

    def turn_off_suction_cb(self, req):
        response = TriggerResponse()
        response.success = True
        if req.arm == 'left':
            self.left_arm.turn_off_suction()
        elif req.arm == 'right':
            self.right_arm.turn_off_suction()
        else:
            rospy.logerr("[Turn Off Suction] No arm named {}".format(req.arm))
            response.success = False
            response.message = "No arm named {}".format(req.arm)
            return response
        response.message = 'Successfully Turn off {} suction'.format(req.arm)
        return response

    def goto_wait_joint_cb(self, req):
        srv = rospy.ServiceProxy("/goto_joints", GotoJoint)
        request = GotoJointRequest()
        request.arm = 'right'
        request.joint = [71.09, -99.14, 10.14, 90.87, 5.60, 25.18, -48.47]
        srv(request)
        return TriggerResponse()

    def goto_scan_joint_cb(self, req):
        srv = rospy.ServiceProxy("/goto_joints", GotoJoint)
        request = GotoJointRequest()
        request.arm = 'right'
        request.joint = [73.23, -101.93, 21.97, 135.4, -70.09, -2.19, -53.44]
        srv(request)
        return TriggerResponse()



if __name__ == '__main__':
    rospy.init_node('yumi_service_bridge_node')
    node = ServiceBridge()
    rospy.spin()
