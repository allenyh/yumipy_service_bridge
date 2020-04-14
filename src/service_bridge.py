#!/usr/bin/env python
import rospy
from std_srvs.srv import Trigger, TriggerResponse
from yumipy import YuMiRobot, YuMiMotionPlanner
from yumipy.yumi_util import message_to_pose, message_to_state
from yumipy_service_bridge.srv import YumipyTrigger, YumipyTriggerResponse, GetPose, GetPoseResponse, \
    GotoPose, GotoPoseResponse, GotoJoint, GotoJointResponse, MoveGripper, MoveGripperResponse, \
    SetZ, SetZResponse, GotoJointRequest, GotoPoseRequest, GotoPoseSync, GotoPoseSyncRequest, \
    SetTool, SetToolResponse


radian2degree = 57.29577951308232

class ServiceBridge:
    def __init__(self):
        self.robot = YuMiRobot(arm_type='remote')
        self.left_arm = self.robot.left
        self.left_arm.calibrate_gripper()
        self.right_arm = self.robot.right
        self.right_arm.calibrate_gripper()
        self._create_services()
        rospy.loginfo("[ServiceBridge] Node is up!")
        self.current_tool_l = 'gripper'
        self.current_tool_r = 'gripper'
        self.tools = {
            'gripper': [0, 0, 136, 0, 0, 0, 1],
            'suction': [63.5, 18.5, 37.5, 0, 0, 0, 1],
            'calibration': [0, 0, 0, 0, 0, 0, 1]
        }
        self.arms_wait_joint = {'left': [-1.53013010964, -2.31535371672, 0.57962382732,
                                         1.8151423679999998, 1.8772760875199999, 0.037873643639999996, 0.36808992828],
                                'right': [1.5475834016399999, -2.3253020931599995, 0.58520888076,
                                          4.45861797432, 1.8971728404, -0.04206243372, -0.34033919399999996]}

    def _create_services(self):
        rospy.Service("~/goto_pose", GotoPose, self.goto_pose_cb)
        rospy.Service("~/goto_pose_plan", GotoPose, self.goto_pose_plan_cb)
        rospy.Service("~/goto_pose_sync", GotoPoseSync, self.goto_pose_sync_cb)
        rospy.Service("~/goto_joints", GotoJoint, self.goto_joints_cb)
        rospy.Service("~/close_gripper", YumipyTrigger, self.close_gripper_cb)
        rospy.Service("~/open_gripper", YumipyTrigger, self.open_gripper_cb)
        rospy.Service("~/move_gripper", MoveGripper, self.move_gripper_cb)
        rospy.Service("~/get_gripper_width", YumipyTrigger, self.get_gripper_width)
        #rospy.Service("/set_speed", , self.set_speed_cb)
        rospy.Service("~/set_tool", SetTool, self.set_tool_cb)
        rospy.Service("~/set_zone", SetZ, self.set_zone_cb)
        rospy.Service("~/reset_home", YumipyTrigger, self.reset_home_cb)
        rospy.Service("~/calibrate_gripper", YumipyTrigger, self.calibrate_gripper_cb)
        rospy.Service("~/turn_on_suction", YumipyTrigger, self.turn_on_suction_cb)
        rospy.Service("~/turn_off_suction", YumipyTrigger, self.turn_off_suction_cb)
        rospy.Service("~/goto_wait_pose", Trigger, self.goto_wait_pose_cb)
        rospy.Service("~/goto_wait_joint", YumipyTrigger, self.goto_wait_joint_cb)

    def goto_pose_cb(self, req):
        response = GotoPoseResponse()
        response.success = False
        if (req.arm != 'left' and req.arm != 'right') or len(req.position) != 3 or len(req.quat) != 4:
            return response
        pose = ''
        for index, p in enumerate(req.position):
            pose += str(p) + ' '
        for q in req.quat:
            pose += str(q) + ' '
        pose = message_to_pose(pose, 'tool')
        if req.arm == 'left':
            ret = self.left_arm.goto_pose(pose, wait_for_res=req.wait_for_res)
        else:
            ret = self.right_arm.goto_pose(pose, wait_for_res=req.wait_for_res)
        if ret is not None:
            response.success = True
        return response

    def goto_pose_plan_cb(self, req):
        response = GotoPoseResponse()
        response.success = False
        if (req.arm != 'left' and req.arm != 'right') or len(req.position) != 3 or len(req.quat) != 4:
            return response
        pose = ''
        for index, p in enumerate(req.position):
            pose += str(p) + ' '
        for q in req.quat:
            pose += str(q) + ' '
        pose = message_to_pose(pose, 'tool')
        if req.arm == 'left':
            ret = self.left_arm.goto_pose_shortest_path(pose, wait_for_res=req.wait_for_res, tool=self.current_tool_l)
        else:
            ret = self.right_arm.goto_pose_shortest_path(pose, wait_for_res=req.wait_for_res, tool=self.current_tool_r)
        if ret is not None:
            response.success = True
        return response

    def goto_pose_sync_cb(self, req):
        response = GotoPoseResponse()
        response.success = False
        if len(req.position_left) != 3 or len(req.quat_left) != 4 or len(req.position_right) != 3 \
                or len(req.quat_right) != 4:
            rospy.logerr('one of the position or quat info is incorrect')
            return response
        pose_left = ''
        for p in req.position_left:
            pose_left += str(p) + ' '
        for q in req.quat_left:
            pose_left += str(q) + ' '
        pose_left = message_to_pose(pose_left, 'tool')
        pose_right = ''
        for p in req.position_right:
            pose_right += str(p) + ' '
        for q in req.quat_right:
            pose_right += str(q) + ' '
        pose_right = message_to_pose(pose_right, 'tool')
        ret = self.robot.goto_pose_sync(pose_left, pose_right)
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
        response = YumipyTriggerResponse()
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
        response = YumipyTriggerResponse()
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
            if req.width <= 0:
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

    def set_tool_cb(self, req):
        if req.arm == 'left':
            arm = self.left_arm
            self.current_tool_l = req.tool
        if req.arm == 'right':
            arm = self.right_arm
            self.current_tool_r = req.tool
        elif req.arm != 'right' and req.arm != 'left':
            rospy.logerr("[Set Tool] No arm named {}".format(req.arm))
        tcp = self.tools[req.tool]
        pose = ''
        for p in tcp:
            pose += str(p) + ' '
        pose_msg = message_to_pose(pose, 'tool')
        arm.set_tool(pose=pose_msg)
        return SetToolResponse()

    def get_gripper_width(self, req):
        response = YumipyTriggerResponse()
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
        response = YumipyTriggerResponse()
        response.success = True
        if req.arm == 'left':
            pose = '364.3 291.6 123.39 0.06213 0.86746 -0.10842 0.48156'
            pose = message_to_pose(pose, 'tool')
            ret = self.left_arm.goto_pose_shortest_path(pose, wait_for_res=True, tool='gripper')
            self.left_arm.reset_home()
        elif req.arm == 'right':
            pose = '381 -314.7 136.97 0.05760 -0.84288 -0.11252 -0.52304'
            pose = message_to_pose(pose, 'tool')
            ret = self.right_arm.goto_pose_shortest_path(pose, wait_for_res=True, tool='gripper')
            self.right_arm.reset_home()
        elif req.arm == 'all':
            # pose = '364.3 291.6 123.39 0.06213 0.86746 -0.10842 0.48156'
            # pose = message_to_pose(pose, 'tool')
            ret = self.left_arm.goto_pose_shortest_path(
                message_to_pose('364.3 291.6 123.39 0.06213 0.86746 -0.10842 0.48156', 'tool'),
                wait_for_res=True, tool='gripper')
            self.left_arm.reset_home()
            # pose = '381 -314.7 136.97 0.05760 -0.84288 -0.11252 -0.52304'
            # pose = message_to_pose(pose, 'tool')
            ret = self.right_arm.goto_pose_shortest_path(
                message_to_pose('381 -314.7 136.97 0.05760 -0.84288 -0.11252 -0.52304', 'tool'),
                wait_for_res=True, tool='gripper')
            self.right_arm.reset_home()
        else:
            rospy.logerr("[Reset Home] No arm named {}".format(req.arm))
            response.success = False
            response.message = "No arm named {}".format(req.arm)
            return response
        response.message = 'Successfully reset {} arm to home'.format(req.arm)
        return response

    def calibrate_gripper_cb(self, req):
        response = YumipyTriggerResponse()
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
        response = YumipyTriggerResponse()
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
        response = YumipyTriggerResponse()
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

    def goto_wait_pose_cb(self, req):
        arms_quat = {'left': [0, 0, -0.9238795, 0.3826834], 'right': [0, 0, 0.9238795, 0.3826834]}
        arms_wait_pose = {'left': [356, 200, 100], 'right': [356, -200, 100]}
        pose = ''
        for p in arms_wait_pose['left']:
            pose += str(p) + ' '
        for q in arms_quat['left']:
            pose += str(q) + ' '
        pose = message_to_pose(pose, 'tool')
        self.left_arm.goto_pose_shortest_path(pose, tool=self.current_tool_l)
        pose = ''
        for p in arms_wait_pose['right']:
            pose += str(p) + ' '
        for q in arms_quat['right']:
            pose += str(q) + ' '
        pose = message_to_pose(pose, 'tool')
        self.right_arm.goto_pose_shortest_path(pose, tool=self.current_tool_r)
        return TriggerResponse()

    def goto_wait_joint_cb(self, req):
        if req.arm == 'left':
            left_joint = self.arms_wait_joint['left']
            state = ""
            for j in left_joint:
                state += str(j*radian2degree) + ' '
            state = message_to_state(state)
            self.left_arm.goto_state(state)
        elif req.arm == 'right':
            right_joint = self.arms_wait_joint['right']
            state = ""
            for j in right_joint:
                state += str(j * radian2degree) + ' '
            state = message_to_state(state)
            self.right_arm.goto_state(state)
        return YumipyTriggerResponse()


if __name__ == '__main__':
    rospy.init_node('yumi_service_bridge_node')
    node = ServiceBridge()
    rospy.spin()
