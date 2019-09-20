from ray.rllib.env import MultiAgentEnv


class MultiAgentAPI(MultiAgentEnv):

    def __init__(self, env):
        self._observation_space = env.observation_space
        self._action_space = env.action_space
        self._agents = env.agents()  # ids
        self._env = env

    def reset(self):
        state_n = self._env.reset()
        
        return dict(zip(self._agents, state_n))

    def step(self, action_dict):
        next_state_n, reward_n, done_n, _ = self._env.step(action_dict)
        
        return dict(self._agents, next_state_n), dict(self._agents, reward_n), dict(self._agents, done_n), {}
