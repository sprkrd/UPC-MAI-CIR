#!/usr/bin/env python

import rospy
from std_msgs.msg import String

from peyetribe import EyeTribe
import time


def talker():
    IP = "192.168.1.135"
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(30) # 30hz (as the eyetribe)

    tracker = EyeTribe()
    tracker.connect(IP)
    tracker.pushmode()
    

    while not rospy.is_shutdown():
        inp = raw_input()
        if inp == "p": # Pick action
		rospy.loginfo("Pick Action in coordinate:")
	elif inp == "d":
		rospy.loginfo("Drop Action in coordinate:")
	else:
		rospy.loginfo("Unknown action")
		continue

	n = tracker.next()
        rospy.loginfo(n)
        data_str = str(n._avg)
        y_coord = data_str.split(";")[1]
        pub.publish(y_coord)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        tracker.pullmode()
        tracker.close()
        pass
