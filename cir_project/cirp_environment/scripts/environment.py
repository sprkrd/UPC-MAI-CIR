#!/usr/bin/python
import roslib
import rospy
import actionlib

from math import pi
from cirp_vision.msg import CirpDetectedPieces
from iri_wam_generic_pickorplace.msg import PickOrPlaceAction, PickOrPlaceGoal

LEFT = 0
MIDDLE = 1
RIGHT = 2
HELD = 3
LAST_D = 4
LAST_L = 5

class EnvironmentError(Exception):
    pass

class Environment:

    REST_POINT = [0.22, 0.000, 0.06]
    RIGHT_CTN_POINT = [0.087, 0.531, -0.07] # REAL
    LEFT_CTN_POINT = [0.126, -0.490, -0.07] # REAL
    # LEFT_CTN_POINT = [-0.125, -0.65, -0.2] # SIMULATION
    # RIGHT_CTN_POINT = [-0.125, 0.65, -0.2] # SIMULATION

    def __init__(self):
        rospy.init_node("environment")
        self._th1 = rospy.get_param("~/th1", default=-0.2)
        self._th2 = rospy.get_param("~/th2", default=0.2)
        self._on_table = None
        self._holding = None
        self._last_r = None
        self._last_l = None
        rospy.Subscriber("pieces", CirpDetectedPieces, self._detected_cb)

    def _detected_cb(self, data):
        if not self._on_table:
            rospy.loginfo("First CirpDetectedPieces message received")
        on_table = [None, None, None]
        for color, point in zip(data.colors, data.positions):
            if point.y < self._th1:
                position = LEFT
            elif point.y < self._th2:
                position = MIDDLE
            else:
                position = RIGHT
            if on_table[position] is not None:
                rospy.logwarn("More than one piece in position " + str(position))
            else:
                on_table[position] = (color, [point.x, point.y, point.z])
        self._on_table = on_table

    def _pick(self, location):
        if self._on_table and self._on_table[location] is not None and self._holding is None:
            color = self._on_table[location][0]
            position = self._on_table[location][1]
            action_client = actionlib.SimpleActionClient("pick_or_place",
                    PickOrPlaceAction)
            rospy.loginfo("Waiting for action server...")
            action_client.wait_for_server()
            goal = PickOrPlaceGoal()
            goal.ini_point = position[0:2] + [position[2]+0.1]
            goal.grasp_point = position # REAL
            # goal.grasp_point = position[0:2] + [position[2]-0.005] # SIMULATION
            goal.end_point = Environment.REST_POINT
            goal.ini_EF_rpy = [pi, 0, pi]
            goal.end_EF_rpy = [pi, 0, pi]
            goal.pick = True
            goal.execute = True
            action_client.send_goal(goal)
            rospy.loginfo("Executing pick action...")
            action_client.wait_for_result(rospy.Duration(60.0))
            result = action_client.get_result()
            if result.successful:
                self._holding = color
        return self.get_current_state()

    def _put(self, where):
        if self._holding is not None:
            color = self._holding
            action_client = actionlib.SimpleActionClient("pick_or_place",
                    PickOrPlaceAction)
            rospy.loginfo("Waiting for action server...")
            action_client.wait_for_server()
            goal = PickOrPlaceGoal()
            if where == LEFT:
                goal.ini_point = list(Environment.LEFT_CTN_POINT)
                goal.grasp_point = Environment.LEFT_CTN_POINT
            else:
                goal.ini_point = list(Environment.RIGHT_CTN_POINT)
                goal.grasp_point = Environment.RIGHT_CTN_POINT
            # goal.ini_point[-1] += 0.2
            goal.end_point = Environment.REST_POINT
            goal.ini_EF_rpy = [pi, 0, pi]
            goal.end_EF_rpy = [pi, 0, pi]
            goal.pick = False
            goal.execute = True
            action_client.send_goal(goal)
            rospy.loginfo("Executing put action...")
            action_client.wait_for_result(rospy.Duration(60.0))
            result = action_client.get_result()
            if result.successful:
                self._holding = None
                if where == LEFT:
                    self._last_l = color
                else:
                    self._last_r = color
        return self.get_current_state()

    def execute_action(self, action):
        if action == "pickL":
            return self._pick(LEFT)
        elif action == "pickM":
            return self._pick(MIDDLE)
        elif action == "pickR":
            return self._pick(RIGHT)
        elif action == "putL":
            return self._put(LEFT)
        elif action == "putR":
            return self._put(RIGHT)
        else:
            raise EnvironmentError("Unknown action: " + action)

    def get_current_state(self):
        if not self._on_table:
            rospy.logwarn("There is no data about the pieces on the table yet")
            on_table = [None, None, None]
        else:
            # in Python3, explicit conversion to list is required
            on_table = map(lambda t: t[0] if t else None, self._on_table)
        state = on_table + [self._holding, self._last_l, self._last_r]
        return state

if __name__ == "__main__":
    env = Environment()
    rospy.sleep(rospy.Duration(1.0))
    # env.execute_action("pickM")
    # env.execute_action("putL")
    # env.execute_action("pickL")
    # env.execute_action("putR")
    # env.execute_action("pickR")
    # env.execute_action("putR")
    quit = False
    while not quit:
        cmd = int(raw_input("Select command: (1) query_state; "
            "(2) pickL; (3) pickM; (4) pickR; (5) putL; (6) putR; (7) exit: "))
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
            print("Shutting down...")
            quit = True
        else:
            rospy.logerr("Unknown cmd: " + str(cmd))

