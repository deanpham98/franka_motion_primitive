import os
import json
import rospy
from python_utils import *
from franka_controllers.msg import HybridControllerState
from franka_motion_primitive.msg import MotionGeneratorState

class FrankaStateSaver:
    def __init__(self):
        rospy.init_node("state_saver")
        self._data = {
            "p": [],
            "pd": [],
            "q": [],
            "qd": [],
            "f": [],
            "f_filter": [],
            "f_ee": [],
            # "jacobian": [],
            # "body_jacobian": []
        }

        # state subscriber
        sub_state_topic = "/hybrid_controller/state"
        self.sub_state = rospy.Subscriber(sub_state_topic, HybridControllerState,
                            self._sub_state_callback)

        # motion gen state
        sub_motion_gen_topic = "/motion_generator/state"
        self.sub_motion_gen_state = rospy.Subscriber(sub_motion_gen_topic,
                                        MotionGeneratorState, self._sub_motion_gen_callback)
        self._record = False

    def _sub_state_callback(self, msg):
        if self._record:
            self._data["p"].append([msg.time, msg.p])
            self._data["pd"].append([msg.time, msg.pd])
            self._data["q"].append([msg.time, msg.q])
            self._data["qd"].append([msg.time, msg.qd])
            # self._data["jacobian"].append([msg.time, msg.jacobian])
            # self._data["body_jacobian"].append([msg.time, msg.body_jacobian])

    def _sub_motion_gen_callback(self, msg):
        if self._record:
            self._data["f"].append([msg.stamp, msg.f])
            self._data["f_filter"].append([msg.stamp, msg.f_filter])
            self._data["f_ee"].append([msg.stamp, msg.f_ee])


    def start_record(self):
        self._record = True
        self._data = {
            "p": [],
            "pd": [],
            "q": [],
            "qd": [],
            "f": [],
            "f_filter": [],
            "f_ee": [],
            # "jacobian": [],
            # "body_jacobian": [],

        }

    def save(self, file_path):
        if not os.path.exists(file_path):
            os.mknod(file_path)
        with open(file_path, "w") as f:
            json.dump(self._data, f)
        self._record = False

if __name__ == '__main__':
    import datetime
    import rospkg
    now = datetime.datetime.now()

    file_name = "data_" + now.strftime("%H_%M_%d.json")

    r = rospkg.RosPack()
    pkg_dir = r.get_path("franka_motion_primitive")
    file = os.path.join(pkg_dir, "data/" + file_name)

    state_saver = FrankaStateSaver()
    raw_input("Press to start record")
    print("start record")
    state_saver.start_record()

    raw_input("press to stop and save")
    state_saver.save(file)
