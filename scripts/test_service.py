from franka_motion_primitive.ros_interface import RosInterface

class TestService:
    def __init__(self):
        self.interface = RosInterface()

    def test_set_init_force(self):
        self.interface.set_init_force()

if __name__ == '__main__':
    test = TestService()
    test.test_set_init_force()
