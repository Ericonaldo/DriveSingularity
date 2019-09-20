import json
import numpy as np

from gym import spaces
from .base import BaseScenario, DEFAULT_OBSERVATION_SHAPE


class Scenario(BaseScenario):
    _rule = dict()  # agent-wise reward function.
    _observation_shape = None
    _action_shape = None

    @property
    def observation_space(self):
        return dict(
            zip(self._observation_shape.keys(), [
                spaces.Box(low=-np.inf, high=+np.inf, shape=shape, dtype=np.float32)
                for shape in self._observation_shape.values()
            ]))

    @property
    def action_space(self):
        # base action space
        return dict(
            zip(self._action_shape.keys(), [
                spaces.Discrete(shape)
                for shape in self._action_shape.values()
            ]))

    def init(self, **kwargs):
        env = kwargs['env']
        agent_ids = env.agents()

        self._observation_shape = dict(
            zip(agent_ids, [DEFAULT_OBSERVATION_SHAPE for _ in agent_ids]))
        self._action_shape = env.action_space  # dict of action space

    def reset_world(self, world):
        world.reset()

    def register_reward_rule(self, world):
        agents = world.agents()

        for agent in agents:
            world.add_listener(agent, [1, 2, 8, 16, 32])  # collision return false

            funcs = {
                1: lambda x: x * 0.2,    # x: float, cosine of lane direction and agent direction
                2: lambda x: x * -2.,   # x: bool, collision or not
                8: lambda x: x * -1.,    # x: bool, reverse or not
                16: lambda x: x * 0.25,    # x: float, high speed ratio
                32: lambda x: x * -0.5,   # x: bool, switch lane or not
            }

            self._rule[agent] = funcs

    def reward(self, agent, event_horizon: list):
        rew = 0.

        assert isinstance(event_horizon, list)

        for event_container in event_horizon:
            for event_key, func in self._rule[agent].items():
                rew += func(
                    json.loads(event_container.get_event(event_key))['value'])

        return rew

    def observation(self, agent, world):
        res = np.asarray(world.get_observation(agent, 84, 84), dtype=np.float32)
        return np.swapaxes(res, 0, 2)  # switch channel and view range

    def done(self, agent, world):
        """ Do not implement it. """
        raise NotImplementedError
