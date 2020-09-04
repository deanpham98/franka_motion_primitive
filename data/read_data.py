import os
import rospkg
import json
import transforms3d.quaternions as Q
import numpy as np
import matplotlib.pyplot as plt
from criros.filters import ButterLowPass

# FILE_NAME = "triangle_data_11_43_07.json"
# FILE_NAME = "square_data_11_21_07.json"
FILE_NAME = "estimate_clearance_11_28_07.json"
DATA_RATE = 500

# calculate [x] of R^3 vector
def cross(x):
    return np.array([[0, -x[2], x[1]],
                     [x[2], 0, -x[0]],
                     [-x[1], x[0], 0]])

def read_data(f):
    with open(f, "r") as p:
        d = json.load(p)
        return d

def filter(x, alpha):
    x_filter = np.zeros_like(x)
    x_filter[0] = x[0]
    for i in range(len(x)-1):
        x_filter[i+1] = alpha*x[i+1] + (1-alpha)*x_filter[i]

    return x_filter

def dft(x, legend=None):
    x = x.T
    N = len(x)
    fig, (ax1, ax2) = plt.subplots(2, 1)
    ax1.plot(range(0, N), x)

    x_freq = np.zeros_like(x)
    for i in range(x.shape[1]):
        x_freq[:, i] = np.fft.fft(x[:, i])
        x_freq[0, i] = 0

    # x_freq[np.argmax(x_freq)] = 0
    ax2.plot(np.array(range(0, N)) / float(N) * 2000, abs(x_freq))
    if legend is not None:
        ax1.legend(legend)

def analyze_force(d):
    f = [k[1] for k in d["f"]]
    f = np.array(f)

    f_filter = [k[1] for k in d["f_filter"]]
    f_filter = np.array(f_filter)

    f_filter_now = filter(f, 0.385)

    N = min(len(f), len(f_filter))

    # f_compare = np.vstack((f[:N, 3], f_filter[:N, 3]))
    f_compare = np.vstack((f[:N, 3], f_filter_now[:N, 3]))

    # dft(f[:, 3])
    # dft(f_filter[:, 3])
    dft(f_compare)
    plt.show()

def base_ee_compare(d):
    idx = 3
    f = [k[1] for k in d["f"]]
    f = np.array(f)
    f_ee = [k[1] for k in d["f_ee"]]
    f_ee = np.array(f_ee)

    N = min(len(f), len(f_ee))
    print(len(f), len(f_ee))

    f_compare = np.vstack((f[:N, idx], f_ee[:N, idx]))
    dft(f_compare, ["f", "f_ee"])
    dft(f[:N, 2].reshape((1, N)))
    plt.show()
    # fig, ax = plt.subplots(1, 1)
    # ax.plot(range(0, len(f)), f[:, 4])
    # ax.plot(range(0, len(f_ee)), f_ee[:, 4])
    # ax.legend(["f", "f_ee"])
    # plt.show()

# transform f_ee to f_0 and compare to f_0_measure
# so O_F_ext_hat_K is actually F^O_O
def transform_ee(d):
    IDX_PLOT = 4
    f_ee = [k[1] for k in d["f_ee"]]
    f_ee = np.array(f_ee)
    tf = np.array([k[0] for k in d["f_ee"]])

    f = [k[1] for k in d["f"]]
    f = np.array(f)

    # ee orientation
    q = [k[1] for k in d["q"]]
    p = [k[1] for k in d["p"]]
    tq = np.array([k[0] for k in d["q"]])

    # since q and f_ee are sampled at different rate.
    # align them before analysed
    tf_align_idx = []
    for t in tq:
        k = np.argmin(np.abs(tf - t))
        tf_align_idx.append(k)
    f_align = f_ee[tf_align_idx]

    # check resized force vs originanl
    # plt.plot(tf - tf[0], f_ee[:, 0])
    # plt.plot(tf[tf_align_idx] - tf[0], f_align[:, 0])
    # plt.show()

    # transform
    f_b = np.zeros_like(f_align)
    # test
    f_b1 = np.zeros_like(f_align)
    for i, qi in enumerate(q):
        pi = p[i]
        R = Q.quat2mat(qi)
        f_b[i, :3] = R.dot(f_align[i, :3])
        f_b[i, 3:] = R.dot(f_align[i, 3:]) + cross(pi).dot(f_b[i, :3])

    N = len(q)
    plt.plot(range(0, N), f_align[:N, IDX_PLOT])
    plt.plot(range(0, N), f_b[:N, IDX_PLOT])
    plt.plot(np.array(range(0, len(f)))*float(N)/len(f), f[:, IDX_PLOT])
    plt.legend(["f_ee", "f_0_transform", "f_0_measure"])
    plt.show()
    # print(len(f_ee), len(q))

def plot_position(d, axis=0):
    p = [k[1] for k in d["p"]]
    p = np.array(p)

    # filter = ButterLowPass(100, 500, 7)
    # # filter.zi = np.zeros((3, 1))
    # p_filter = filter(p[:, axis].reshape(1, len(p)))
    # print(p_filter.shape)

    fig, ax = plt.subplots()
    ax.plot(range(len(p)), p[:, axis])
    # ax.plot(range(p_filter.shape[1]), p_filter.T)
    plt.show()

def plot_position_error(d, trans_dof=True):
    p = [k[1] for k in d["p"]]
    tp = [k[0] for k in d["p"]]
    p = np.array(p)
    pd = [k[1] for k in d["pd"]]
    tpd = [k[0] for k in d["pd"]]
    pd = np.array(pd)

    p_align = np.zeros_like(pd)
    tp_align_idx = []
    for t in tpd:
        k = np.argmin(np.abs(np.array(tp) - t))
        tp_align_idx.append(k)
    p_align = p[tp_align_idx]

    fig, (ax1, ax2, ax3) = plt.subplots(3, 1)
    if trans_dof:
        ax1.plot(range(len(p_align)), p_align[:, 0] - pd[:, 0])
        ax2.plot(range(len(p_align)), p_align[:, 1] - pd[:, 1])
        ax3.plot(range(len(p_align)), p_align[:, 2] - pd[:, 2])
    else:
        pass

    # ax.plot(range(p_filter.shape[1]), p_filter.T)
    plt.show()


def estimate_clearance(d):
    p = [k[1] for k in d["p"]]
    p = np.array(p)

    f = [k[1] for k in d["f"]]
    f = np.array(f)

    # print(len(p), len(f))

    # clearance x
    maxx_idx = np.argmax(p[:, 0])
    minx_idx = np.argmin(p[:, 0])
    # print(p[maxx_idx, 2], p[minx_idx, 2])
    print(f[maxx_idx, 0], f[minx_idx, 0], )

    for idx, fi in enumerate(f):
        if np.abs(fi[0]) <= 1e-2:
            k = (f[maxx_idx, 0] - fi[0]) / (p[maxx_idx, 0] - p[idx, 0])
            print(k)
            break


    # clearance y
    maxx_idx = np.argmax(p[:, 1])
    minx_idx = np.argmin(p[:, 1])
    # print(p[maxx_idx, 2], p[minx_idx, 2])

    print(5. / 26900)
    print("clearance x: {}".format(max(p[:, 0]) - min(p[:, 0])))
    # print("clearance y: {}".format(max(p[:, 1]) - min(p[:, 1])))

if __name__ == '__main__':
    r = rospkg.RosPack()
    pkg_dir = r.get_path("franka_motion_primitive")
    file = os.path.join(pkg_dir, "data/" + FILE_NAME)

    d = read_data(file)
    # analyze_force(d)
    # base_ee_compare(d)
    # transform_ee(d)
    # plot_position(d, axis=1)
    # estimate_clearance(d)
    # plot_position_error(d)
