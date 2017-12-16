import numpy as np

class User:

	# actions:
	actions = ["pick1", "pick2", "pick3", "putL", "putR"]

	def __init__(self, env):
		# TODO
		self.env = env
		self.rewards = env.get_real_reward_function()

	def poll_next_action(self):
		return self.actions[np.random.randint(0, 5)]

	def get_random_action(self):
		return self.actions[np.random.randint(0, 5)]

	def get_good_action(self):
		state_index = self.env.get_current_state_index()
		action_rewards = self.rewards[state_index, :]
		if np.count_nonzero(action_rewards) != 0:
			optimal_action = np.argmax(action_rewards)
		else:
			state = self.env.get_current_state()
			if state[3] == 0:
				optimal_action = np.random.randint(0, 3)
			else:
				optimal_action = np.random.randint(3, 5)
		return self.actions[optimal_action]
