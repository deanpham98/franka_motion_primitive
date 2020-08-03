import os
import json
import rospkg
import transforms3d.quaternions as Q
from openravepy import *
import numpy as np
from read_data import read_data, cross

FILE_NAME = "data_12_35_01.json"

class RaveFranka:
    def __init__(self, render=False):
        e = Environment()
        if render:
            e.SetViewer("qtcoin")
        e.Load("worlds/empty_world.xml")
        self.robot = e.GetRobot("franka_panda")
        # end-effector
        self.manip = self.robot.SetActiveManipulator('panda_hand')
        self.robot.SetActiveDOFs(self.manip.GetArmIndices())

        # load ik database
        self.ikmodel = databases.inversekinematics.InverseKinematicsModel(self.robot,\
                    iktype=IkParameterization.Type.Transform6D)

        if not self.ikmodel.load():
            self.ikmodel.autogenerate()

    def find_closest_solution(self, list_solution):
        qseed = self.robot.GetActiveDOFValues()
        weights = self.robot.GetDOFWeights(self.manip.GetArmIndices())
        if len(list_solution) > 0:
          distances = [sum(weights*(qseed-qsol)**2) for qsol in list_solution]
          closest = np.argmin(distances)
          return list_solution[closest]
        else:
          return None

    # find ik solution from position p and quaternion q
    def ik(self, p, q):
        R = Q.quat2mat(q)
        Tee = self.manip.GetEndEffectorTransform()
        Tee[:3, :3] = R.copy()
        Tee[:3, 3] = p.copy()
        solutions = self.manip.FindIKSolutions(
            Tee, IkFilterOptions.CheckEnvCollisions)
        theta = self.find_closest_solution(solutions)
        return theta

    def get_jacobian(self, theta):
        self.robot.SetActiveDOFValues(theta)
        J = np.zeros((6,7))
        J[:3, :] = self.manip.CalculateJacobian()
        J[3:, :] = self.manip.CalculateAngularVelocityJacobian()
        return J

def compare_jacobian(d):
    # compare jacobian at idx
    COMPARE_IDX = 100

    J = [k[1] for k in d["jacobian"]]
    tJ = np.array([k[0] for k in d["jacobian"]])
    J = np.array(J)

    Jb = [k[1] for k in d["body_jacobian"]]
    tJb = np.array([k[0] for k in d["body_jacobian"]])
    Jb = np.array(J)

    p = [k[1] for k in d["p"]]
    tp = np.array([k[0] for k in d["p"]])
    p = np.array(p)

    q = [k[1] for k in d["q"]]
    tq = np.array([k[0] for k in d["q"]])
    q = np.array(q)

    # align signal
    t = [tJ, tJb, tp, tq]
    N_sig = (len(tJ), len(tJb), len(tp), len(tq))
    main_sig, N = np.argmin(N_sig), np.min(N_sig)
    J_al = np.zeros((N, 42))
    Jb_al = np.zeros((N, 42))
    p_al = np.zeros((N, 3))
    q_al = np.zeros((N, 4))

    for i, ti in enumerate(t[main_sig]):
        if main_sig != 0:
            idx = np.argmin(np.abs(tJ - ti))
            J_al[i, :] = J[idx, :]
        else:
            J_al[i, :] = J[i, :]

        if main_sig != 1:
            idx = np.argmin(np.abs(tJb - ti))
            Jb_al[i, :] = Jb[idx, :]
        else:
            Jb_al[i, :] = Jb[i, :]

        if main_sig != 2:
            idx = np.argmin(np.abs(tp - ti))
            p_al[i, :] = p[idx, :]
        else:
            p_al[i, :] = p[i, :]

        if main_sig != 3:
            idx = np.argmin(np.abs(tq - ti))
            q_al[i, :] = q[idx, :]
        else:
            q_al[i, :] = q[i, :]

    # franka = RaveFranka()
    J1 = J_al[COMPARE_IDX, :].reshape((7, 6)).T
    Jb1 = J_al[COMPARE_IDX, :].reshape((7, 6)).T
    p1 = p_al[COMPARE_IDX, :]
    q1 = q_al[COMPARE_IDX, :]
    R1 = Q.quat2mat(q1)
    # theta = franka.ik(p1, q1)
    # print(theta)
    # J_rave = franka.get_jacobian(theta)
    # print(J1, J_rave)
    # print((J1 - J_rave) / J_rave * 100)

    Jtrans1 = np.zeros_like(J1)
    Jtrans1[:3, :] = R1.dot(Jb1[:3, :])
    Jtrans1[3:, :] = R1.dot(Jb1[3:, :])

    print(Jtrans1 - J1)

    Jtrans2 = np.zeros_like(J1)
    Jtrans2[3:, :] = R1.dot(Jb1[3:, :])
    Jtrans2[:3, :] = R1.dot(Jb1[:3, :]) + cross(x).dot(Jtrans2[3:, :])
    print(Jtrans2 - J1)

if __name__ == '__main__':
    r = rospkg.RosPack()
    pkg_dir = r.get_path("franka_motion_primitive")
    file = os.path.join(pkg_dir, "data/" + FILE_NAME)

    d = read_data(file)
    compare_jacobian(d)
