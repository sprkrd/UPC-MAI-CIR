from Environment import Environment
from User import User
from RL import train_Q

import time
import numpy as np

# state variable indices:
c1, c2, c3, arm, last_l, last_r = list(range(0, 6))

# state variable values:
empty, red, blue, green = list(range(0, 4))

# actions:
actions = ["pick1", "pick2", "pick3", "putL", "putR"]


def is_warnable(current_state, next_action, last_action, warned_last_step):
	if next_action == last_action and warned_last_step:
		print('Skip warning since action was repeated')
		return False
	s = current_state
	a = next_action
	exp_val = Q[s[0], s[1], s[2], s[3], s[4], s[5], actions.index(a)]
	possible_vals = Q[s[0], s[1], s[2], s[3], s[4], s[5], :]
	best_val = np.max(possible_vals)
	quality = exp_val / best_val
	print(possible_vals)
	print(quality)
	if quality < 0.9:
		return True
	else:
		return False


def reward_estimate_from_trajectory(trajectory):
	# TODO
	pass




# # # # # # # # # # 
# # # # # # # # # #
# execution starts here:
# # # # # # # # # #
# # # # # # # # # #



print('Getting state action values from optimal policy with some random exploration')
Q = train_Q(iterations=200000, traj_len=100, alpha_min=0.5)
print(Q[1, 2, 3, 0, :, 0, :])
np.save('Qs', np.copy(Q))
Q = np.load('Qs.npy')

# print(Q[1, 2, 3, 0, :, 0, :])


# data structure for estimated reward of each state-action pair
# 4^6 possible states (6 state variables with 4 values each)
# 5 possible actions
# reward_estimate = np.zeros((4 ** 6, 5))


env = Environment()
env.reset()
user = User(env)


current_state = env.get_current_state()
last_action = None
warned_last_step = False

while True:

	next_action = user.poll_next_action()

	print('- ' * 10)
	print("state: \t" + str(current_state))
	print("action: " + str(next_action))

	if is_warnable(current_state, next_action, last_action, warned_last_step):

		env.warn()
		warned_last_step = True

	else:

		current_state = env.execute_action(next_action)
		warned_last_step = False

	last_action = next_action
	time.sleep(1)












