#!/usr/bin/env python

import rospy
from std_msgs.msg import String

from peyetribe import EyeTribe
import time

from cir_user.srv import UserAction, UserActionResponse

TH1 = 1280*(1.0/3)
TH2 = 1280*(2.0/3)

def talker():
    def service_callback(req):
        inp = raw_input("Press p or d: ").strip()
        while inp not in ("p", "d"):
            inp = raw_input("Unknown action {}. Press p or d: ".format(inp)).strip()
        rospy.loginfo("x_coord=" + str(x_coord))
        if x_coord < TH1:
            direction = "L"
        elif x_coord < TH2:
            direction = "M"
        else:
            direction = "R"
        if inp == "p": # Pick action
            rospy.loginfo("Pick Action in coordinate:")
            action = "pick" + direction
        elif inp == "d":
            rospy.loginfo("Drop Action in coordinate:")
            action = "put" + direction
        if action == "putM": # Invalid action
            action = ""
        return UserActionResponse(action=action)
    IP = "192.168.43.137"
    x_coord = 0
    rospy.init_node('talker', anonymous=True)
    rospy.Service("poll_action", UserAction, service_callback)
    # pub = rospy.Publisher('chatter', String, queue_size=10)
    # rate = rospy.Rate(30) # 30hz (as the eyetribe)

    tracker = EyeTribe()
    tracker.connect(IP)
    tracker.pullmode()

    while not rospy.is_shutdown():
        n = tracker.next()
        data_str = str(n._avg)
        x_coord = float(data_str.split(";")[0])

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        tracker.pullmode()
        tracker.close()
        pass
