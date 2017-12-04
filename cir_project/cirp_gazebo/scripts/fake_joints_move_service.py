#! /usr/bin/env python

# This script simulates the joints_move service from the real controller.

import roslib # ; roslib.load_manifest('iri_wam_controller')
import rospy
import actionlib

from control_msgs.srv      import QueryTrajectoryState
from iri_common_drivers_msgs.srv import QueryJointsMovement, QueryJointsMovementRequest
from iri_common_drivers_msgs.srv import QueryJointsMovementResponse
from control_msgs.msg      import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg     import JointTrajectory, JointTrajectoryPoint

def get_current_joints():
  query_state = rospy.ServiceProxy(query_joints_name, QueryTrajectoryState)
  res = query_state(rospy.Time.now())
  return res.position

def handle_joints_move(req):
  goal = FollowJointTrajectoryGoal()
  goal.trajectory.joint_names = joint_names
  current_joints = get_current_joints()
  max_inc = max(map(lambda (x,y):abs(x-y),zip(req.positions,current_joints)))
  # if abs(max_inc) < 0.0174:
    # return QueryJointsMovementResponse(success=1)
  j_traj = JointTrajectoryPoint()
  duration = max_inc/req.velocity
  j_traj.time_from_start = rospy.Duration(duration)
  j_traj.positions = req.positions
  j_traj.velocities = [0]*7
  j_traj.accelerations = [0]*7
  j_traj.effort = [0.0]*7
  goal.trajectory.points.append(j_traj)

  try:
    # Creates the SimpleActionClient, passing the type of the action
    # (FollowJointTrajectoryAction) to the constructor.
    action_client = actionlib.SimpleActionClient(action_client_name,
        FollowJointTrajectoryAction)
    # Waits until the action server has started up and started
    # listening for goals.
    action_client.wait_for_server()
    # Send goal
    action_client.send_goal(goal)
    rospy.sleep(duration)
  except rospy.ROSInterruptException as e:
    print(e)
  return QueryJointsMovementResponse(success=1)


################################

if __name__ == '__main__':
  rospy.init_node('fake_services')
  robot = rospy.get_param("~robot",default="iri_wam")
  base_frame = "{}_link_base".format(robot)
  joint_names = ["{robot}_joint_{joint}".format(robot=robot,joint=joint) for joint in range(1,8)]
  action_client_name = "/{robot}/{robot}_controller/follow_joint_trajectory".format(robot=robot)
  query_joints_name = "/{robot}/{robot}_controller/query_state".format(robot=robot)
  joints_move_srv_name = "/{robot}/{robot}_controller/joints_move".format(robot=robot)
  joints_move = rospy.Service(joints_move_srv_name,QueryJointsMovement,handle_joints_move)
  rospy.spin()

