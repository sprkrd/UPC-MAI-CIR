#!/usr/bin/env python

import rospy
from std_msgs.msg import String

from peyetribe import EyeTribe
import time

from cir_user.srv import UserAction, UserActionResponse

TH1 = 1280*(1.0/3)
TH2 = 1280*(2.0/3)

IP = "192.168.101.72"

class CameraUserServer:

    def __init__(self):
        rospy.init_node("talker")
        rospy.Service("poll_action", UserAction, self._action_cb)
        self._next_action = None
        # self._tracker = EyeTribe()
        # self._tracker.connect(IP)
        # self._tracker.pullmode()

    def _action_cb(self, req):
        rate = rospy.Rate(10)
        action = self._next_action
        while action is None:
            rate.sleep()
            action = self._next_action
            self._next_action = None
        return UserActionResponse(action=action)

    def set_next_action(self):
        inp = raw_input("Action: ")
        self._next_action = inp
        # inp = raw_input("Press p or d: ").strip()
        # while inp not in ("p", "d"):
            # inp = raw_input("Unknown action {}. Press p or d: ".format(inp)).strip()
        # data_str = str(self._tracker.next()._avg)
        # x_coord = float(data_str.split(";")[0])
        # rospy.loginfo("x_coord=" + str(x_coord))
        # if x_coord < TH1:
            # direction = "L"
        # elif x_coord < TH2:
            # direction = "M"
        # else:
            # direction = "R"
        # if inp == "p": # Pick action
            # rospy.loginfo("Pick Action in coordinate:")
            # action = "pick" + direction
        # elif inp == "d":
            # rospy.loginfo("Drop Action in coordinate:")
            # action = "put" + direction
        # if action == "putM": # Invalid action
            # action = ""
        # self._next_action = action

    def shutdown(self):
        self._tracker.close()

if __name__ == '__main__':
    try:
        user = CameraUserServer()
        while True:
            user.set_next_action()
    except rospy.ROSInterruptException:
        pass
    finally:
        print "Shutting down server..."
        user.shutdown()

