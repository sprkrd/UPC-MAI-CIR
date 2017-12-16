#!/usr/bin/python
import sys
import rospy

from cirp_user.srv import UserAction

class UserClient:

    def __init__(self):
        print "Waiting for service..."
        rospy.wait_for_service("poll_action")
        print "Service active. Starting client."
        self._user_action = rospy.ServiceProxy("poll_action", UserAction)

    def poll_next_action(self):
        resp = self._user_action()
        return resp.action

if __name__ == "__main__":
    user = UserClient()
    quit = False
    while not quit:
        cmd = raw_input("Type quit to exit. Anything else to poll next action: ").strip()
        if (cmd == "quit"):
            quit = True
        else:
            print(user.poll_next_action())

