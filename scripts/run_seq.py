import os
import time
import datetime
import rospkg
import numpy as np
import transforms3d.quaternions as Q
from ros_interface import RosInterface
from state_saver import FrankaStateSaver

# initial controller gains
KP0 = np.array([1000.]*3 + [60.]*3)
KD0 = 2*np.sqrt(KP0)

# hole position adn orientation error
HOLE_POSX_ERROR = 0.  # mm
HOLE_POSY_ERROR = 0.  # mm
HOLE_ROTX_ERROR = 0. # deg
HOLE_ROTY_ERROR = 0. # deg
HOLE_ROTZ_ERROR = 0.  # deg

HOLE_SHAPE = "round"
HOLE_DEPTH = 0.02

if HOLE_SHAPE == "round":
    T_HOLE = np.array([0.997444,0.0633262,-0.0328016,0,0.063392,-0.997979,0.000968749,0,-0.0326746,-0.00304569,-0.999461,0,0.529789,-0.087157,0.15136,1]).reshape((4, 4)).T

elif HOLE_SHAPE == "square":
    T_HOLE = np.array([[0.774692,0.63229,-0.00649177,0,\
                        0.632206,-0.774705,-0.0113309,0,\
                        -0.0121939,0.00467394,-0.999915,0,\
                        0.53053,0.0671758,0.143978,1]]).reshape((4, 4)).T

elif HOLE_SHAPE == "triangle":
    T_HOLE = np.array([[0.997716,-0.0608612,-0.0289602,0,\
                        -0.0605536,-0.99809,0.0113826,0,\
                        -0.0295982,-0.00960317,-0.999516,0,\
                        0.534134,0.125966,0.143978,1]]).reshape((4, 4)).T

# STEP 1
TILT_ANGLE = -7.5 # deg
# triangle
# TILT_ANGLE = -3 # deg
S1 = 0.2  # speed factor

# step 2
S2 = 0.01   # speed facotr
FT2 = 5.    # threshold force
T2 = 5.     # t max

# step 3
S3 = 0.02
FZD3 = 3
FT3 = 5.
T3 = 5.

# step 4
S4 = 0.015
FXD4 = 5.
FZD4 = 5.
FT4 = 0.7
T4 = 10.

# step 5
KD5 = np.array([0.01, 0.01, 0.01, 0.5, 0.5, 0.3])
# for square
# KD5 = np.array([0.01, 0.01, 0.01, 0.3, 0.3, 0.3])
FZD5 = 8.
ZT = T_HOLE[2, 3] - HOLE_DEPTH*0.95
T5 = 10.

class Sequence(object):
    """
    T_hole: true pose of the hole
    """
    def __init__(self,
                 T_hole):
        self.ros_interface = RosInterface()
        self.T_hole_true = T_hole
        self.T_hole_hat = self.T_hole_true.copy()

    def set_hole_pose_error(self, hole_pos_error, hole_rot_error):
        # orientation_error
        axe = hole_rot_error
        thetae = np.linalg.norm(axe)
        if thetae >= 1e-5:
            axe = axe / thetae
            qe = Q.axangle2quat(axe, thetae)
            Re = Q.quat2mat(qe)
            self.T_hole_hat[:3, :3] = Re.dot(self.T_hole_true[:3, :3])

        # position error
        self.T_hole_hat[:3, 3] += hole_pos_error


    def step0(self, kp, kd, hole_pos_error, hole_rot_error):
        # set hole pose error
        self.set_hole_pose_error(hole_pos_error, hole_rot_error)
        # set gain
        kp = np.array([1000., 1000., 1000., 80., 80., 50.])
        kd = 2*np.sqrt(kp)
        self.ros_interface.set_gain(kp, kd)
        time.sleep(0.5)

        # move to above the hole
        p0 = self.T_hole_hat[:3, 3].copy()
        p0[2] += 0.005
        q0 = Q.mat2quat(self.T_hole_hat[:3, :3])

        self.ros_interface.move_to_pose(p0, q0, 0.2)
        time.sleep(0.5)

        # calib initial force
        self.ros_interface.set_init_force()

    # approach the hole
    def step1(self, tilt_angle, speed_factor=0.2):
        p1 = self.T_hole_hat[:3, 3].copy()
        p1[2] += 0.01
        # for square
        # p1[1] +=0.01
        p1[0] += 0.005

        axis = np.array([0., 1., 0])
        # for square peg
        # axis = np.array([-1., 1., 0])
        qtilt = Q.axangle2quat(axis, tilt_angle)
        qcur = Q.mat2quat(self.T_hole_hat[:3, :3])
        q1 = Q.qmult(qtilt, qcur)
        self.ros_interface.move_to_pose(p1, q1, speed_factor=speed_factor)

    # move down in the z direction
    def step2(self, speed_factor=0.01, f_thresh=5., timeout=2.):
        cmd = self.ros_interface.get_constant_velocity_cmd()
        cmd.constant_velocity_param.speed_factor = speed_factor
        cmd.constant_velocity_param.direction = np.array([0, 0, -1., 0, 0, 0])

        # move for 1 sec
        cmd.constant_velocity_param.timeout = timeout
        cmd.constant_velocity_param.f_thresh = f_thresh

        # raw_input("press enter")
        self.ros_interface.run_primitive(cmd)

    # slide x direction
    def step3(self, speed_factor=0.01, fzd=3., f_thresh=5., timeout=5):
        cmd = self.ros_interface.get_constant_velocity_cmd()
        cmd.constant_velocity_param.speed_factor = speed_factor
        cmd.constant_velocity_param.direction = np.array([-1, 0, 0., 0, 0, 0])
        # for square
        # cmd.constant_velocity_param.direction = np.array([-1, -1, 0., 0, 0, 0])
        cmd.constant_velocity_param.fd = np.array([0, 0, -fzd, 0 , 0,0 ])
        # move for 1 sec
        cmd.constant_velocity_param.timeout = timeout
        cmd.constant_velocity_param.f_thresh = f_thresh
        # raw_input("press enter")
        self.ros_interface.run_primitive(cmd)

    # rotate until contact
    def step4(self, speed_factor, fxd, fzd, f_thresh, timeout):
        cmd = self.ros_interface.get_constant_velocity_cmd()
        cmd.constant_velocity_param.speed_factor = speed_factor
        cmd.constant_velocity_param.direction = np.array([0, 0, 0., 0, 1, 0])
        cmd.constant_velocity_param.fd = np.array([-fxd, 0, -fzd, 0 , 0,0 ])
        # move for 1 sec
        cmd.constant_velocity_param.timeout = timeout
        cmd.constant_velocity_param.f_thresh = f_thresh
        # raw_input("press enter")
        self.ros_interface.run_primitive(cmd)

    # track 0 torque and force
    def step5(self, kd, fzd, z_thresh, timeout):
        ##  set controller gain
        kp_controller= np.array([500., 500., 500., 50., 50., 50.])
        kd_controller = np.array([10.]*3 + [5.]*3)
        self.ros_interface.set_gain(kp_controller, kd_controller)
        time.sleep(0.5)

        cmd = self.ros_interface.get_admittance_motion_cmd()
        cmd.admittance_motion_param.kd = kd
        cmd.admittance_motion_param.timeout = timeout
        cmd.admittance_motion_param.fd = np.array([0, 0, -fzd, 0, 0, 0])
        cmd.admittance_motion_param.z_thresh = z_thresh
        # raw_input("press enter")
        self.ros_interface.run_primitive(cmd)

    # run sequence with a specific params
    def run_sequence(self, params):
        start = time.time()
        kp = params["kp0"]
        kd = params["kd0"]
        hole_pos_error = params["hole_pos_error"]
        hole_rot_error = params["hole_rot_error"]
        self.step0(kp, kd, hole_pos_error, hole_rot_error)

        tilt_angle = params["tilt_angle"]*np.pi /180
        s = params["s1"]
        raw_input("press to run step 1")
        self.step1(tilt_angle, s)

        s = params["s2"]
        f_thresh = params["ft2"]
        timeout = params["t2"]
        raw_input("press to run step 2")
        self.step2(s, f_thresh, timeout)

        s = params["s3"]
        fzd = params["fzd3"]
        f_thresh = params["ft3"]
        timeout = params["t3"]
        raw_input("press to run step 3")
        self.step3(s, fzd, f_thresh, timeout)

        s = params["s4"]
        fxd = params["fxd4"]
        fzd = params["fzd4"]
        f_thresh = params["ft4"]
        timeout = params["t4"]
        raw_input("press to run step 4")
        self.step4(s, fxd, fzd, f_thresh, timeout)

        kd = params["kd_admittance"]
        fzd = params["fzd5"]
        z_thresh = params["z_thresh"]
        timeout = params["t5"]
        raw_input("press to run step 5")
        self.step5(kd, fzd, z_thresh, timeout)

        print("run time: {}".format(time.time() - start))

class TaskFrameSequence(Sequence):
    def __init__(self, T_hole):
        # super(TaskFrameSequence, self).__init__(T_hole)
        super(TaskFrameSequence, self).__init__(T_hole)

    def set_hole_pose_error(self, p, q):
        super(TaskFrameSequence, self).set_hole_pose_error(p, q)
        self.tf_pos = self.T_hole_hat[:3, 3]
        self.hole_quat = Q.mat2quat(self.T_hole_hat[:3, :3])
        qx = np.array([np.cos(np.pi/2), np.sin(np.pi/2), 0, 0])
        self.tf_quat = Q.qmult(self.hole_quat, qx)

    def step0(self, kp, kd, hole_pos_error, hole_rot_error):
        # set hole pose error
        self.set_hole_pose_error(hole_pos_error, hole_rot_error)
        # set gain
        kp = np.array([1000., 1000., 1000., 80., 80., 50.])
        kd = 2*np.sqrt(kp)
        self.ros_interface.set_gain(kp, kd)
        time.sleep(0.5)

        # move to above the hole
        p0 = np.array([0, 0, 0.005])
        q0 = np.array([0., 1., 0., 0])

        self.ros_interface.move_to_pose(p0, q0, 0.2, tf_pos=self.tf_pos, tf_quat=self.tf_quat)
        time.sleep(0.5)

        # calib initial force
        self.ros_interface.set_init_force()

    # approach the hole
    def step1(self, tilt_angle, speed_factor=0.2):
        p1 = np.zeros(3)
        p1[2] += 0.01
        # for square
        # p1[1] +=0.01
        p1[0] += 0.005

        axis = np.array([0., 1., 0])
        # for square peg
        # axis = np.array([-1., 1., 0])
        qtilt = Q.axangle2quat(axis, tilt_angle)
        qtarget = np.array([0., 1., 0, 0])
        q1 = Q.qmult(qtilt, qtarget)
        self.ros_interface.move_to_pose(p1, q1, speed_factor=speed_factor, tf_pos=self.tf_pos, tf_quat=self.tf_quat)

    # move down in the z direction
    def step2(self, speed_factor=0.01, f_thresh=5., timeout=2.):
        cmd = self.ros_interface.get_constant_velocity_cmd()
        cmd.constant_velocity_param.speed_factor = speed_factor
        cmd.constant_velocity_param.direction = np.array([0, 0, -1., 0, 0, 0])

        # move for 1 sec
        cmd.constant_velocity_param.timeout = timeout
        cmd.constant_velocity_param.f_thresh = f_thresh

        cmd.constant_velocity_param.task_frame.pos = self.tf_pos
        cmd.constant_velocity_param.task_frame.quat = self.tf_quat

        # raw_input("press enter")
        self.ros_interface.run_primitive(cmd)

    # slide x direction
    def step3(self, speed_factor=0.01, fzd=3., f_thresh=5., timeout=5):
        cmd = self.ros_interface.get_constant_velocity_cmd()
        cmd.constant_velocity_param.speed_factor = speed_factor
        cmd.constant_velocity_param.direction = np.array([-1, 0, 0., 0, 0, 0])
        # for square
        # cmd.constant_velocity_param.direction = np.array([-1, -1, 0., 0, 0, 0])
        cmd.constant_velocity_param.fd = np.array([0, 0, -fzd, 0 , 0,0 ])
        # move for 1 sec
        cmd.constant_velocity_param.timeout = timeout
        cmd.constant_velocity_param.f_thresh = f_thresh
        cmd.constant_velocity_param.task_frame.pos = self.tf_pos
        cmd.constant_velocity_param.task_frame.quat = self.tf_quat

        # raw_input("press enter")
        self.ros_interface.run_primitive(cmd)

    # rotate until contact
    def step4(self, speed_factor, fxd, fzd, f_thresh, timeout):
        cmd = self.ros_interface.get_constant_velocity_cmd()
        cmd.constant_velocity_param.speed_factor = speed_factor
        cmd.constant_velocity_param.direction = np.array([0, 0, 0., 0, 1, 0])
        cmd.constant_velocity_param.fd = np.array([-fxd, 0, -fzd, 0 , 0,0 ])
        # move for 1 sec
        cmd.constant_velocity_param.timeout = timeout
        cmd.constant_velocity_param.f_thresh = f_thresh
        cmd.constant_velocity_param.task_frame.pos = self.tf_pos
        cmd.constant_velocity_param.task_frame.quat = self.tf_quat

        # raw_input("press enter")
        self.ros_interface.run_primitive(cmd)

    # track 0 torque and force
    def step5(self, kd, fzd, z_thresh, timeout):

        cmd = self.ros_interface.get_admittance_motion_cmd()
        cmd.admittance_motion_param.kd = kd
        cmd.admittance_motion_param.timeout = timeout
        cmd.admittance_motion_param.fd = np.array([0, 0, -fzd, 0, 0, 0])
        cmd.admittance_motion_param.z_thresh = z_thresh
        # raw_input("press enter")
        self.ros_interface.run_primitive(cmd)

# def run_sequence():
#     ros_interface = RosInterface()
#
#     Tee = np.array([0.999276,-0.0322603,-0.0196635,0,\
#         -0.0323633,-0.999454,-0.00494357,0,\
#         -0.0194936,0.00557648,-0.999794,0,\
#         0.530069,-0.0850753,0.150997,1]).reshape((4, 4)).T
#
#     # add artificial error
#     axe = np.array([HOLE_ROTX_ERROR, HOLE_ROTY_ERROR, 0])* np.pi/180
#     thetae = np.linalg.norm(axe)
#     if thetae >= 1e-5:
#         axe = axe / thetae
#         qe = Q.axangle2quat(axe, thetae)
#         Re = Q.quat2mat(qe)
#         Tee[:3, :3] = Re.dot(Tee[:3, :3])
#
#     # step 0 calibration
#     kp = np.array([1000., 1000., 1000., 80., 80., 50.])
#     kd = 2*np.sqrt(kp)
#     ros_interface.set_gain(kp, kd)
#     time.sleep(0.5)
#
#     p0 = Tee[:3, 3].copy()
#     p0[2] += 0.005
#     q0 = Q.mat2quat(Tee[:3, :3])
#
#     ros_interface.move_to_pose(p0, q0, 0.2)
#     time.sleep(1.)
#
#     # calib initial force
#     ros_interface.set_init_force()
#
#     # step 1
#     p1 = Tee[:3, 3].copy()
#     p1[2] += 0.01
#     p1[0] += 0.005
#
#     angle = -0.13
#     axis = np.array([0., 1., 0])
#     qtilt = Q.axangle2quat(axis, angle)
#     qcur = Q.mat2quat(Tee[:3, :3])
#     q1 = Q.qmult(qtilt, qcur)
#
#     speed_factor=0.2
#     # raw_input("press enter")
#     ros_interface.move_to_pose(p1, q1, speed_factor=speed_factor)
#
#     # step 2
#     cmd = ros_interface.get_constant_velocity_cmd()
#     cmd.constant_velocity_param.speed_factor = 0.01
#     cmd.constant_velocity_param.direction = np.array([0, 0, -1., 0, 0, 0])
#
#     # move for 1 sec
#     cmd.constant_velocity_param.timeout = 2.
#     cmd.constant_velocity_param.f_thresh = 5.
#
#     # raw_input("press enter")
#     ros_interface.run_primitive(cmd)
#     # step 3
#     cmd = ros_interface.get_constant_velocity_cmd()
#     cmd.constant_velocity_param.speed_factor = 0.01
#     cmd.constant_velocity_param.direction = np.array([-1, 0, 0., 0, 0, 0])
#     cmd.constant_velocity_param.fd = np.array([0, 0, -3, 0 , 0,0 ])
#     # move for 1 sec
#     cmd.constant_velocity_param.timeout = 5.
#     cmd.constant_velocity_param.f_thresh = 5.
#     # raw_input("press enter")
#     ros_interface.run_primitive(cmd)
#
#     # step 4 - rotate util contact
#     cmd = ros_interface.get_constant_velocity_cmd()
#     cmd.constant_velocity_param.speed_factor = 0.015
#     cmd.constant_velocity_param.direction = np.array([0, 0, 0., 0, 1, 0])
#     cmd.constant_velocity_param.fd = np.array([-5, 0, -5, 0 , 0,0 ])
#     # move for 1 sec
#     cmd.constant_velocity_param.timeout = 15.
#     cmd.constant_velocity_param.f_thresh = 0.7
#     # raw_input("press enter")
#     ros_interface.run_primitive(cmd)
#
#     # step 5 - admittance motion
    # ##  set controller gain
    # kp_controller= np.array([500., 500., 500., 50., 50., 50.])
    # kd_controller = np.array([10.]*3 + [5.]*3)
    # ros_interface.set_gain(kp_controller, kd_controller)
    # time.sleep(0.5)
#
#     # with admittance
#     kd = np.array([0.015, 0.015, 0.015, 0.9, 0.9, 0.5])
#     # only push down
#     # kd = np.zeros(6)
#
#     fd = np.array([0, 0, -8.] + [0.]*3)
#     timeout = 10.
#     cmd = ros_interface.get_admittance_motion_cmd()
#     cmd.admittance_motion_param.kd = kd
#     cmd.admittance_motion_param.timeout = timeout
#     cmd.admittance_motion_param.fd = fd
#     cmd.admittance_motion_param.z_thresh = Tee[2, 3] - 0.02*0.95
#     # raw_input("press enter")
#     ros_interface.run_primitive(cmd)

class Experiment(object):
    def __init__(self):
        self.state_saver = FrankaStateSaver(run_node=False)

        self.params = dict(
            kp0=KP0, kd0=KD0,
            hole_pos_error=np.array([HOLE_POSX_ERROR, HOLE_POSY_ERROR, 0.])/1000.,
            hole_rot_error=np.array([HOLE_ROTX_ERROR, HOLE_ROTY_ERROR, HOLE_ROTZ_ERROR])* np.pi/180,
            s1=S1, tilt_angle = TILT_ANGLE,
            s2=S2, ft2=FT2, t2=T2,
            s3=S3, fzd3=FZD3, ft3=FT3, t3=T3,
            s4=S4, fxd4=FXD4, fzd4=FZD4, ft4=FT4, t4=T4,
            kd_admittance=KD5, fzd5=FZD5, z_thresh=ZT, t5=T5
        )

        # self.seq = Sequence(T_HOLE)
        self.seq = TaskFrameSequence(T_HOLE)

    def get_save_dir(self, name):
        # create save file
        import datetime
        import rospkg
        now = datetime.datetime.now()

        file_name = name + now.strftime("_%H_%M_%d.json")

        r = rospkg.RosPack()
        pkg_dir = r.get_path("franka_motion_primitive")
        file = os.path.join(pkg_dir, "data/" + file_name)
        return file

    def run_sequence(self, record=False):
        run_name = HOLE_SHAPE + "_data"
        save_dir = self.get_save_dir(run_name)
        if record:
            self.state_saver.start_record()

        self.seq.run_sequence(self.params)
        if record:
            self.state_saver.save(save_dir)

    def test_range_velocity(self):
        # v = [0.01, 0.02, 0.03, 0.05]
        v = [0.07, 0.08]
        for vi in v:
            kp = self.params["kp0"]
            kd = self.params["kd0"]
            hole_pos_error = self.params["hole_pos_error"]
            hole_rot_error = self.params["hole_rot_error"]
            self.seq.step0(kp, kd, hole_pos_error, hole_rot_error)

            tilt_angle = self.params["tilt_angle"]*np.pi /180
            s = self.params["s1"]
            # raw_input("press to run step 1")
            self.seq.step1(tilt_angle, s)

            s = self.params["s2"]
            f_thresh = self.params["ft2"]
            timeout = self.params["t2"]
            # raw_input("press to run step 2")
            self.seq.step2(s, f_thresh, timeout)

            s = self.params["s3"]
            fzd = self.params["fzd3"]
            f_thresh = self.params["ft3"]
            timeout = self.params["t3"]
            # raw_input("press to run step 3")
            self.seq.step3(s, fzd, f_thresh, timeout)
            #
            s = vi
            fxd = self.params["fxd4"]
            fzd = self.params["fzd4"]
            f_thresh = self.params["ft4"]
            timeout = self.params["t4"]
            # raw_input("press to run step 4")
            self.seq.step4(s, fxd, fzd, f_thresh, timeout)

            self.seq.ros_interface.move_up(timeout=1.)

    def estimate_clearance(self):
        file_name = self.get_save_dir("estimate_clearance")

        # first insert peg into hole
        # self.seq.run_sequence(self.params)

        # start record data
        self.state_saver.start_record()

        # apply x force
        cmd = self.seq.ros_interface.get_constant_velocity_cmd()
        cmd.constant_velocity_param.speed_factor = 0.5
        cmd.constant_velocity_param.direction = np.array([-0, 0, 0., 0, 0, 0])
        cmd.constant_velocity_param.fd = np.array([10, 0, -8, 0 , 0,0 ])
        # move for 1 sec
        cmd.constant_velocity_param.timeout = 2.
        cmd.constant_velocity_param.f_thresh = 10.
        # raw_input("press enter")
        self.seq.ros_interface.run_primitive(cmd)

        # apply -x force
        cmd.constant_velocity_param.fd = np.array([-10, 0, -8, 0 , 0,0 ])
        # raw_input("press enter")
        self.seq.ros_interface.run_primitive(cmd)

        # apply y force
        cmd.constant_velocity_param.fd = np.array([0, 10, -8, 0, 0, 0])
        self.seq.ros_interface.run_primitive(cmd)

        # apply -y force
        cmd.constant_velocity_param.fd = np.array([0, -10, -8, 0, 0, 0])
        self.seq.ros_interface.run_primitive(cmd)

        # savfe data
        self.state_saver.save(file_name)

if __name__ == '__main__':
    exp = Experiment()
    # exp.run_sequence(record=True)
    exp.test_range_velocity()

    # exp.estimate_clearance()
