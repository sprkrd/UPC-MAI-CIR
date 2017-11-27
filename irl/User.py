import numpy as np

class User:

	# actions:
	actions = ["pick1", "pick2", "pick3", "putL", "putR"]

	def __init__(self):
		# TODO
		pass

	def get_next_action(self):
		return self.actions[np.random.randint(0, 5)]
