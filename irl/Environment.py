import numpy as np
from tqdm import tqdm

# state variable indices:
c1, c2, c3, arm, last_l, last_r = list(range(0, 6))

# state variable values:
empty, red, blue, green = list(range(0, 4))

# actions:
actions = ["pick1", "pick2", "pick3", "putL", "putR"]

initial_state = [1, 2, 3, 0, 0, 0]

class Environment:

	def __init__(self):
		self.reset()

	def get_current_state(self):
		return np.copy(self.state)

	def get_current_state_index(self):
		state = self.get_current_state()
		result = 0
		for i in range(0, 6):
			result += state[-1 - i] * (4 ** i)
		return result

	def reset(self):

		self.state = initial_state
		self.rewards = []
		self.fullness_l = 0
		self.fullness_r = 0

	def get_initial_state(self):
		return np.copy(self.initial_state)

	def execute_action(self, action):

		self.rewards.append(self.get_reward(self.state, action))
		# print("total reward: " + str(np.sum(self.rewards)))

		if action not in actions:
			raise ValueError("Illegal action: " +str(action))

		if action.startswith("pick") and self.state[arm] == empty:
			# fill arm with correct color:
			self.state[arm] = self.state[actions.index(action)]
			# put new random disk on now empty spot:
			self.state[actions.index(action)] = np.random.randint(1, 4)

		if action.startswith("put") and self.state[arm] != empty:
			# set corresponding putL/putR
			if action == "putL":
				self.state[last_l] = self.state[arm]
				self.fullness_l += 1
			else:
				self.state[last_r] = self.state[arm]
				self.fullness_r += 1
			# empty arm:
			self.state[arm] = empty

		# empty buckets when too full:
		if self.fullness_l > 15:
			self.state[last_l] = empty
			self.fullness_l = 0
		if self.fullness_r > 15:
			self.state[last_r] = empty
			self.fullness_r = 0
		

		return self.get_current_state()


	def get_reward(self, state, action):

		reward = 0

		if state[c1] == state[c3] and state[arm] != empty and action == "putL":
			reward += 30

		if state[arm] == red and action == "putL":
			reward += 80

		if state[arm] == green and action == "putR":
			reward += 60

		if state[arm] == blue and action == "putR":
			reward += 40

		if state[arm] == red and state[last_l] == green and action == "putL":
			reward += -200

		if state[arm] == green and (green in [state[c1], state[c2], state[c3]]) and action == "putR":
			reward += -200

		return reward


	def warn(self):
		# TODO
		print("IRL warns the user")


	def get_real_reward_function(self):

		# reward[c1, c2, c3, arm, last_l, last_r, action]
		rewards = np.zeros((4, 4, 4, 4, 4, 4, 5))

		for c1_v in range(4):
			for c2_v in range(4):
				for c3_v in range(4):
					for arm_v in range(4):
						for last_l_v in range(4):
							for last_r_v in range(4):
								for action_v in range(len(actions)):
									rewards[c1_v, c2_v, c3_v, arm_v, last_l_v, last_r_v, action_v]  \
										= self.get_reward([c1_v, c2_v, c3_v, arm_v, last_l_v, last_r_v], actions[action_v])

		return rewards


