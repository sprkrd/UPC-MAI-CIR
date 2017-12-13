from Environment import Environment
from User import User

import time
import numpy as np


def is_warnable(current_state, next_action, last_action):
	# TODO
	return False


def reward_estimate_from_trajectory(trajectory):
	# TODO
	pass



# # # # # # # # # # 
# # # # # # # # # #
# execution starts here:
# # # # # # # # # #
# # # # # # # # # #

env = Environment()
user = User(env)


# data structure for estimated reward of each state-action pair
# 4^6 possible states (6 state variables with 4 values each)
# 5 possible actions
reward_estimate = np.zeros((4 ** 6, 5))


current_state = env.get_current_state()
last_action = None
warned_last_step = False

while True:

	next_action = user.get_good_action()

	print('- ' * 10)
	print("state: \t" + str(current_state))
	print("action: " + str(next_action))

	if is_warnable(current_state, next_action, last_action):

		env.warn()
		warned_last_step = True

	else:

		env.execute_action(next_action)
		warned_last_step = False

	last_action = next_action
	time.sleep(1)








