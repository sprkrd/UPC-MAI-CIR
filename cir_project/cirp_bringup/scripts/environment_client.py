#!/usr/bin/python
import sys
import rospy

from cirp_environment.srv import CirpEnvironmentAction

class EnvironmentClient:

    def __init__(self):
        print "Waiting for service..."
        rospy.wait_for_service("execute_action")
        print "Service active. Starting client."
        self._env_action = rospy.ServiceProxy("execute_action", CirpEnvironmentAction)
    
    def warn(self, msg="Warning! Are you sure you want to execute this action"):
        self._env_action("say "+msg)

    def get_current_state(self):
        resp = self._env_action("getCurrentState")
        return resp.state

    def execute_action(self, action):
        resp = self._env_action(action)
        return resp.state

if __name__ == "__main__":
    env = EnvironmentClient()
    quit = False
    while not quit:
        cmd = int(raw_input("Select command: (1) query_state; "
            "(2) pickL; (3) pickM; (4) pickR; (5) putL; (6) putR; (7) say; (8) exit: "))
        if cmd == 1:
            print(env.get_current_state())
        elif cmd == 2:
            print(env.execute_action("pickL"))
        elif cmd == 3:
            print(env.execute_action("pickM"))
        elif cmd == 4:
            print(env.execute_action("pickR"))
        elif cmd == 5:
            print(env.execute_action("putL"))
        elif cmd == 6:
            print(env.execute_action("putR"))
        elif cmd == 7:
            msg = raw_input("Enter message: ")
            env.execute_action("say {}".format(msg))
        elif cmd == 8:
            print("Shutting down...")
            quit = True
        else:
            rospy.logerr("Unknown cmd: " + str(cmd))

