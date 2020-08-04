import time
import numpy as np
import transforms3d.quaternions as Q
from ros_interface import RosInterface

HOLE_ROTX_ERROR = -1. # deg
HOLE_ROTY_ERROR = 1. # deg

def run_node1(Thole, init_angle, speed_factor=0.2):
    pass

def run_sequence():
    ros_interface = RosInterface()

    Tee = np.array([0.999276,-0.0322603,-0.0196635,0,\
        -0.0323633,-0.999454,-0.00494357,0,\
        -0.0194936,0.00557648,-0.999794,0,\
        0.530069,-0.0850753,0.150997,1]).reshape((4, 4)).T

    # add artificial error
    axe = np.array([HOLE_ROTX_ERROR, HOLE_ROTY_ERROR, 0])* np.pi/180
    thetae = np.linalg.norm(axe)
    if thetae >= 1e-5:
        axe = axe / thetae
        qe = Q.axangle2quat(axe, thetae)
        Re = Q.quat2mat(qe)
        Tee[:3, :3] = Re.dot(Tee[:3, :3])

    # step 0 calibration
    kp = np.array([1000., 1000., 1000., 80., 80., 50.])
    kd = 2*np.sqrt(kp)
    ros_interface.set_gain(kp, kd)
    time.sleep(0.5)

    p0 = Tee[:3, 3].copy()
    p0[2] += 0.005
    q0 = Q.mat2quat(Tee[:3, :3])

    raw_input("press enter")
    ros_interface.move_to_pose(p0, q0, 0.2)
    time.sleep(1.)
    ros_interface.set_init_force()

    # step 1
    p1 = Tee[:3, 3].copy()
    p1[2] += 0.01
    p1[0] += 0.005

    angle = -0.13
    axis = np.array([0., 1., 0])
    qtilt = Q.axangle2quat(axis, angle)
    qcur = Q.mat2quat(Tee[:3, :3])
    q1 = Q.qmult(qtilt, qcur)

    speed_factor=0.2
    raw_input("press enter")
    ros_interface.move_to_pose(p1, q1, speed_factor=speed_factor)

    # step 2
    cmd = ros_interface.get_constant_velocity_cmd()
    cmd.constant_velocity_param.speed_factor = 0.01
    cmd.constant_velocity_param.direction = np.array([0, 0, -1., 0, 0, 0])

    # move for 1 sec
    cmd.constant_velocity_param.timeout = 2.
    cmd.constant_velocity_param.f_thresh = 5.

    raw_input("press enter")
    ros_interface.run_primitive(cmd)
    # step 3
    cmd = ros_interface.get_constant_velocity_cmd()
    cmd.constant_velocity_param.speed_factor = 0.01
    cmd.constant_velocity_param.direction = np.array([-1, 0, 0., 0, 0, 0])
    cmd.constant_velocity_param.fd = np.array([0, 0, -3, 0 , 0,0 ])
    # move for 1 sec
    cmd.constant_velocity_param.timeout = 5.
    cmd.constant_velocity_param.f_thresh = 5.
    raw_input("press enter")
    ros_interface.run_primitive(cmd)

    # step 4 - rotate util contact
    cmd = ros_interface.get_constant_velocity_cmd()
    cmd.constant_velocity_param.speed_factor = 0.015
    cmd.constant_velocity_param.direction = np.array([0, 0, 0., 0, 1, 0])
    cmd.constant_velocity_param.fd = np.array([-5, 0, -5, 0 , 0,0 ])
    # move for 1 sec
    cmd.constant_velocity_param.timeout = 15.
    cmd.constant_velocity_param.f_thresh = 0.7
    raw_input("press enter")
    ros_interface.run_primitive(cmd)

    # step 5 - admittance motion
    ##  set controller gain
    kp_controller= np.array([500., 500., 500., 50., 50., 50.])
    kd_controller = np.array([10.]*3 + [5.]*3)
    ros_interface.set_gain(kp_controller, kd_controller)
    time.sleep(0.5)

    # with admittance
    # kd = np.array([0.015, 0.015, 0.015, 0.9, 0.9, 0.5])
    # only push down
    kd = np.zeros(6)

    fd = np.array([0, 0, -8.] + [0.]*3)
    timeout = 10.
    cmd = ros_interface.get_admittance_motion_cmd()
    cmd.admittance_motion_param.kd = kd
    cmd.admittance_motion_param.timeout = timeout
    cmd.admittance_motion_param.fd = fd
    cmd.admittance_motion_param.z_thresh = Tee[2, 3] - 0.02*0.95
    raw_input("press enter")
    ros_interface.run_primitive(cmd)

if __name__ == '__main__':
    run_sequence()
