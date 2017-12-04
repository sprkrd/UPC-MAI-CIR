import numpy as np

# state variable indices:
c1, c2, c3, arm, last_l, last_r = list(range(0, 6))

# state variable values:
empty, red, blue, green = list(range(0, 4))

# actions:
actions = ["pick1", "pick2", "pick3", "putL", "putR"]

class Environment:

	def __init__(self):

		initial_state = [1, 2, 3, 0, 0, 0]
		self.state = initial_state

		self.rewards = []

	def get_current_state(self):
		return self.state

	def execute_action(self, action):

		self.rewards.append(self.get_reward(action))
		print("total reward: " + str(np.sum(self.rewards)))

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
			else:
				self.state[last_r] = self.state[arm]
			# empty arm:
			self.state[arm] = empty

		return self.state


	def get_reward(self, action):

		reward = 0

		if self.state[c1] == self.state[c3] and action == "putL":
			reward += 10

		if self.state[arm] == red and action == "putL":
			reward += 8

		if self.state[arm] == green and action == "putR":
			reward += 6

		if self.state[arm] == blue and action == "putR":
			reward += 4

		if self.state[arm] == red and self.state[last_l] == green and action == "putL":
			reward += -20

		if self.state[arm] == green and (green in [self.state[c1], self.state[c2], self.state[c3]]) and action == "putR":
			reward -= 20

		return reward


	def warn(self):
		# TODO
		print("IRL warns the user")


