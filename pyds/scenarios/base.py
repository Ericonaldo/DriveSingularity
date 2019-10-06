DEFAULT_OBSERVATION_SHAPE = (4, 84, 84, 3)


class BaseScenario(object):
    def __init__(self, *args, **kwargs):
        self.time = 0

    @property
    def observation_space(self):
        raise NotImplementedError

    @property
    def action_space(self):
        raise NotImplementedError

    def init(self, **kwargs):
        raise NotImplementedError

    def reset_world(self, world):
        raise NotImplementedError

    def reward(self, agent, world):
        raise NotImplementedError

    def observation(self, agent, world):
        raise NotImplementedError

    def done(self, agent, world):
        raise NotImplementedError
