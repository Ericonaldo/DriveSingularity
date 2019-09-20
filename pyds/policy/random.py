from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import logging
import numpy as np

from gym.spaces import Box


class Policy(object):
    def __init__(self, obs_space, action_space, model_config, name):
        self.obs_space = obs_space
        self.action_space = action_space
        self.name = name

        self.action_dim = np.product(action_space)

    def act(self, state):
        raise NotImplementedError


class RandomPolicy(Policy):
    def __init__(self, obs_space, action_space, model_config, name):
        super(RandomPolicy, self).__init__(obs_space, action_space,
                                           model_config, name)

    def act(self, state):
        return np.random.choice(self.action_dim, 1)
