import numpy as np
from ros_interface import RosInterface

class TestService:
    def __init__(self):
        self.interface = RosInterface()

    def test_set_init_force(self):
        self.interface.set_init_force()

    def test_set_gain(self):
        kp = np.array([0., 1000., 1000.] + [50.]*3)
        kd = 2*np.sqrt(kp)
        self.interface.set_gain(kp, kd)

if __name__ == '__main__':
    test = TestService()
    # test.test_set_init_force()
    test.test_set_gain()
