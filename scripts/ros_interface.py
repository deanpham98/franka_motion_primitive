import json
import rospy
import numpy as np
from franka_motion_primitive.msg import\
    RunPrimitiveCommand, PrimitiveType, MoveToPoseParam,\
    ConstantVelocityParam, AdmittanceMotionParam

from franka_motion_primitive.srv import *

from franka_controllers.msg import HybridControllerState, Gain
class RosInterface:
    def __init__(self):
        rospy.init_node("ros_interface")

        # primitive publish
        self.pub_prime_ = rospy.Publisher("/motion_generator/run_primitive", RunPrimitiveCommand, queue_size=1)

        # set gain publisher
        self.pub_set_gain_ = rospy.Publisher(
            "hybrid_controller/gain_config", Gain, queue_size=1)

        # state subscriber
        sub_state_topic = "/hybrid_controller/state"
        self.sub_state = rospy.Subscriber(sub_state_topic, HybridControllerState,
                            self._sub_state_callback)

        # service
        self.serv_set_init_force = rospy.ServiceProxy("/motion_generator/set_initial_force", SetInitialForce)
        self.serv_run_primitive = rospy.ServiceProxy("/motion_generator/run_primitive", RunPrimitive)

        self._state = {
            "t": None,
            "p": None,
            "pd": None,
            "q": None,
            "qd": None}
        # wait until subscriber is on
        while self._state["t"] is None:
            rospy.sleep(0.01)

        rospy.sleep(1.)


    def _sub_state_callback(self, msg):
        self._state["t"] = msg.time
        self._state["p"] = np.array(msg.p)
        self._state["pd"] = np.array(msg.pd)
        self._state["q"] = np.array(msg.q)
        self._state["qd"] = np.array(msg.qd)

    def set_init_force(self):
        print("start calling service")
        rospy.wait_for_service("/motion_generator/set_initial_force")
        try:
            res = self.serv_set_init_force()
            if res.success == 1:
                print("Set init force success")
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
        print("end calling service")

    def set_gain(self, kp, kd):
        gain_msg = Gain()
        gain_msg.kp = kp
        gain_msg.kd = kd
        gain_msg.kDefineDamping = 1
        self.pub_set_gain_.publish(gain_msg)

    def publish(self, cmd):
        self.pub_prime_.publish(cmd)

    def get_pos(self):
        return self._state["p"].copy()

    def get_quat(self):
        return self._state["q"].copy()

    # DEPRECATED
    def get_move_to_pose_cmd(self):
        self.cmd = RunPrimitiveRequest()
        self.cmd.type = PrimitiveType.MoveToPose
        p = MoveToPoseParam()
        p.task_frame.pos = np.zeros(3)
        p.task_frame.quat = np.array([1., 0., 0., 0.])

        p.target_pose.pos = self.get_pos()
        p.target_pose.quat = self.get_quat()

        p.speed_factor = 0.1
        p.timeout = 5.
        p.fd = np.zeros(6)

        self.cmd.time = rospy.Time.now().to_sec()
        self.cmd.move_to_pose_param = p
        return self.cmd

    def get_constant_velocity_cmd(self):
        self.cmd = RunPrimitiveRequest()
        self.cmd.type = PrimitiveType.ConstantVelocity

        p = ConstantVelocityParam()
        p.task_frame.pos = np.zeros(3)
        p.task_frame.quat = np.array([1., 0, 0, 0])
        p.speed_factor = 0.
        p.direction = np.array([0., 0., 1., 0, 0, 0])
        p.timeout = 1.
        p.f_thresh = 0.
        p.fd = np.array([0. ,0. ,0., 0., 0., 0.])

        self.cmd.time = rospy.Time.now().to_sec()
        self.cmd.constant_velocity_param = p
        return self.cmd

    def get_admittance_motion_cmd(self):
        self.cmd = RunPrimitiveRequest()
        self.cmd.type = PrimitiveType.AdmittanceMotion

        p = AdmittanceMotionParam()
        p.kd = np.zeros(6)
        p.fd = np.zeros(6)
        p.timeout = 1
        self.cmd.time = rospy.Time.now().to_sec()
        self.cmd.admittance_motion_param = p
        return self.cmd

    def run_primitive(self, cmd):
        rospy.wait_for_service("/motion_generator/run_primitive")
        try:
            res = self.serv_run_primitive(cmd)
            return res.status, res.time
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            return None, None

    # move to pose defined in base frame
    # quat [w, x, y, z]
    # NOTE not ensure that the motion will be executed
    def move_to_pose(self, pos, quat, speed_factor=0.1, tf_pos=np.zeros(3), tf_quat=np.array([1., 0, 0, 0])):
        cmd = self.get_move_to_pose_cmd()
        cmd.move_to_pose_param.target_pose.pos = pos
        cmd.move_to_pose_param.target_pose.quat = quat
        cmd.move_to_pose_param.speed_factor = speed_factor
        cmd.move_to_pose_param.task_frame.pos = tf_pos
        cmd.move_to_pose_param.task_frame.quat = tf_quat

        self.run_primitive(cmd)

    def move_up(self, speed_factor=0.05, timeout=2.):
        cmd = self.get_constant_velocity_cmd()
        cmd.constant_velocity_param.speed_factor = speed_factor
        cmd.constant_velocity_param.direction = np.array([0, 0., 1., 0, 0, 0])

        # move for 1 sec
        cmd.constant_velocity_param.timeout = timeout
        cmd.constant_velocity_param.f_thresh = 100.
        self.run_primitive(cmd)
