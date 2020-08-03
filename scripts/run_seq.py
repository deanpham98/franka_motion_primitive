import transforms3d.quaternions as Q
from ros_interface import RosInterface

def run_node1(Thole, init_angle, speed_factor=0.2):
    pass

def run_sequence():
    ros_interface = RosInterface()

    Tee = np.array([0.999276,-0.0322603,-0.0196635,0,\
        -0.0323633,-0.999454,-0.00494357,0,\
        -0.0194936,0.00557648,-0.999794,0,\
        0.530069,-0.0850753,0.150997,1]).reshape((4, 4)).T

    # step 1
    p1 = Tee[:3, 3].copy()
    p1[2] += 0.01
    p1[0] += 0.005

    angle = -0.175
    axis = np.array([0., 1., 0])
    qtilt = Q.axangle2quat(axis, angle)
    qcur = Q.mat2quat(Tee[:3, :3])
    q1 = Q.qmult(qtilt, qcur)

    speed_factor=0.2
    raw_input()
    ros_interface.move_to_pose(p1, q1, speed_factor=speed_factor)

    # step 2
    cmd = ros_interface.get_constant_velocity_cmd()
    cmd.constant_velocity_param.speed_factor = 0.01
    cmd.constant_velocity_param.direction = np.array([0, 0, -1., 0, 0, 0])

    # move for 1 sec
    cmd.constant_velocity_param.timeout = 2.
    cmd.constant_velocity_param.f_thresh = 5.

    raw_input()
    ros_interface.publish(cmd)
    # step 3
    cmd = ros_interface.get_constant_velocity_cmd()
    cmd.constant_velocity_param.speed_factor = 0.01
    cmd.constant_velocity_param.direction = np.array([-1, 0, 0., 0, 0, 0])
    cmd.constant_velocity_param.fd = np.array([0, 0, -8, 0 , 0,0 ])
    # move for 1 sec
    cmd.constant_velocity_param.timeout = 2.
    cmd.constant_velocity_param.f_thresh = 5.
    raw_input()
    ros_interface.publish(cmd)

    # step 4 - rotate util contact
    cmd = ros_interface.get_constant_velocity_cmd()
    cmd.constant_velocity_param.speed_factor = 0.01
    cmd.constant_velocity_param.direction = np.array([0, 0, 0., 0, 1, 0])
    cmd.constant_velocity_param.fd = np.array([0, 0, -8, 0 , 0,0 ])
    # move for 1 sec
    cmd.constant_velocity_param.timeout = 2.
    cmd.constant_velocity_param.f_thresh = 0.2
    raw_input()
    ros_interface.publish(cmd)

    # step 5 - admittance motion
    kd = np.array([0.01, 0.01, 0.01, 0.2, 0.2, 0.2])
    fd = np.array([0, 0, -8.] + [0.]*3)
    timeout = 10.
    cmd = ros_interface.get_admittance_motion_cmd()
    cmd.admittance_motion_param.kd = kd
    cmd.admittance_motion_param.timeout = timeout
    cmd.admittance_motion_param.fd = fd
    cmd.admittance_motion_param.z_thresh = Tee[3, 2] - 0.02*0.9
    raw_input()
    ros_interface.publish(cmd)


if __name__ == '__main__':
    run_sequence()
