import json
import numpy as np

from gym import spaces
from .base import BaseScenario, DEFAULT_OBSERVATION_SHAPE


class Scenario(BaseScenario):
    _rule = dict()
    _observation_shape = None
    _action_shape = None

    @property
    def observation_space(self):
        return dict(zip(self._observation_shape.keys(), [
            spaces.Box(low=-np.inf, high=+np.inf, shape=shape) for shape in self._observation_shape]))

    @property
    def action_space(self):
        return dict(zip(self._action_shape.keys(), [spaces.Discrete(shape) for shape in self._action_shape.values()]))

    def init(self, **kwargs):
        env = kwargs['env']
        agent_ids = env.agents()

        self._observation_shape = dict(
            zip(agent_ids, [DEFAULT_OBSERVATION_SHAPE for _ in agent_ids]))
        self._action_shape = env.action_space  # dict of action space

    def register_reward_rule(self, world):
        """ Events:
            1 - keep straight (return: cosin)
            2 - collision (return: bool)
            16 - high speed ratio (return: float, range [0, 1])
            32 - switch lane or not ()
        """
        agents = world.agents()

        for agent in agents:
            world.add_listener(agent, [1, 2, 16, 32])

            funcs = {
                1: lambda x: (x - 1.) * 0.5,
                2: lambda x: x * -3.,
                16: lambda x: (x - 0.) * 0.5,
                32: lambda x: x * -0.25
            }

            self._rule[agent] = funcs

    def reward(self, agent, event_horizon: list):
        rew = 0.

        for event_container in event_horizon:
            for e_key, func in self._rule[agent].items():
                rew += func(json.loads(event_container.get_event(e_key)))
        
        return rew

    def observation(self, agent, world):
        res = np.asarray(world.get_observation(agent, 84, 84), dtype=np.float32)
        return np.swapaxes(res, 0, 2)  # switch channel and view range

    def done(self, agent, world):
        """ Do not implement it. """
        raise NotImplementedError