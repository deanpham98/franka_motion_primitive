from state_saver import FrankaStateSaver

class TestStateSaver:
    def __init__(self):
        self.state_saver = FrankaStateSaver()

    def test_save_file(self):
        import os
        import rospkg
        r = rospkg.RosPack()
        pkg_dir = r.get_path("franka_motion_primitive")
        file = os.path.join(pkg_dir, "scripts/test.json")

        self.state_saver.start_record()
        self.state_saver.save(file)

    def test_script(self):
        raw_input("Press to start record")
        print("start record")
        self.state_saver.start_record()

        raw_input("press to stop and save")
        self.state_saver.save("test")

if __name__ == '__main__':
    test =TestStateSaver()
    # test.test_save_file() # PASS
    test.test_script()
