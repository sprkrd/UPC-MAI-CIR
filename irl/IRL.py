from Environment import Environment
from User import User

import time

def is_warnable(current_state, next_action, last_action):
	# TODO
	return False

env = Environment()
user = User()

current_state = env.get_current_state()
last_action = None
warned_last_step = False

while True:

	next_action = user.get_next_action()

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








