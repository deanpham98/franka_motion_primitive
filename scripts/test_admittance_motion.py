from ros_interface import RosInterface
import numpy as np
import transforms3d.quaternions as Q

class TestAdmittanceMotion:
    def __init__(self):
        self.ros_interface = RosInterface()

    def test_null_cmd(self):
        cmd = self.ros_interface.get_admittance_motion_cmd()
        cmd.admittance_motion_param.timeout = 10
        self.ros_interface.publish(cmd)

    def test_interaction(self):

        # target_pose = np.array([[0.980334, 0.197291, -0.00146722, 0.530611],
        #             [0.19729, -0.980335, -0.000872652, -0.0848445],
        #             [-0.00161053, 0.000566023, -0.999999, 0.15113],
        #             [0 , 0, 0, 1]])
        #
        # # NOTE in base frame
        # pos = target_pose[:3, 3]
        # pos[2]+=0.01
        # pos[1]+=0.015
        # pos[0]+=0.015
        # quat = Q.mat2quat(target_pose[:3, :3])
        # # move to appropriate pose
        # self.ros_interface.move_to_pose(pos, quat, 0.1)

        # reset compensation force here

        kd = np.array([0.01, 0.01, 0.01, 0.07, 0.07, 0.0])
        fd = np.array([0.]*6)
        timeout = 1000.
        cmd = self.ros_interface.get_admittance_motion_cmd()
        cmd.admittance_motion_param.kd = kd
        cmd.admittance_motion_param.timeout = timeout
        cmd.admittance_motion_param.fd = fd
        cmd.admittance_motion_param.z_thresh = -1.
        self.ros_interface.publish(cmd)

if __name__ == '__main__':
    test = TestAdmittanceMotion()
    # test.test_null_cmd()

    test.test_interaction()
