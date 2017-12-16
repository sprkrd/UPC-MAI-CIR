from Environment import Environment
import numpy as np
from tqdm import tqdm
import time
from collections import Counter

c1, c2, c3, arm, last_l, last_r = list(range(0, 6))
empty, red, blue, green = list(range(0, 4))
actions = ["pickL", "pickM", "pickR", "putL", "putR"]


def updateQ(Q, s, a, r, sn, gamma, alpha=0.05):
    old_value = Q[s[c1], s[c2], s[c3], s[arm], s[last_l], s[last_r], actions.index(a)]
    optimal_fut_value = np.max(Q[sn[c1], sn[c2], sn[c3], sn[arm], sn[last_l], sn[last_r], :])
    update = (1.0 - alpha) * old_value + alpha * (r + gamma * optimal_fut_value)
    Q[s[c1], s[c2], s[c3], s[arm], s[last_l], s[last_r], actions.index(a)] = update



def train_Q(iterations=10000, traj_len=100, alpha=1.0, alpha_decay=0.999,  \
          alpha_min=0.05, gamma=0.95, exp_decay=0.999, exp_min=0.5):
    gamma = 0.95
    Q = np.ones((4, 4, 4, 4, 4, 4, 5))
    exp = 1.0
    for it in tqdm(range(iterations)):
        env = Environment()
        s = env.get_current_state()
        for step in range(traj_len):
            if np.random.uniform(0, 1) <= max(exp, exp_min):
                a = actions[np.random.randint(0, 5)]
            else:
                a = actions[np.argmax(Q[s[c1], s[c2], s[c3], s[arm], s[last_l], s[last_r], :])]
            r = env.get_reward(s, a)
            sn = env.execute_action(a)
            updateQ(Q, s, a, r, sn, gamma, max(alpha, alpha_min))
            s = sn
            alpha = alpha * alpha_decay
            exp = exp * exp_decay
    return Q