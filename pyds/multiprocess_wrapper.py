from multiprocessing import Pipe, Process
from ray.rllib.env import MultiAgentEnv

from pyds import engine
from pyds import scenarios
import numpy as np
import time


client_id = 0


def engine_handler(conn, env_setup, scenario):
    # print(env_setup)
    env = engine.Environment(env_setup, scenario=scenarios.load(scenario)())
    episode = 0

    while True:
        mess = conn.recv()

        if mess[0] == "step:action":
            action_dict = mess[1]

            assert isinstance(action_dict, dict)

            # convert key to str
            action_dict = dict(zip(map(int, action_dict.keys()),
                              action_dict.values()))

            next_state_n, reward_n, done_n, _ = env.step(action_dict)

            conn.send(["step:feedback", next_state_n, reward_n, done_n])
        elif mess[0] == "reset:call":
            # print("[INFO] episode #{}".format(episode))
            episode += 1
            state_n = env.reset(episode, mess[1])
            conn.send(["reset:feedback", state_n])
        elif mess[0] == "render:on":
            env.turn_on_render()
            conn.send(["render:on:feedback"])
        elif mess[0] == "render:off":
            env.turn_off_render()
            conn.send(["render:off:feedback"])
        elif mess[0] == "render:call":
            env.render()
            conn.send(["render:feedback"])
        elif mess[0] == "agent_alive:check":
            conn.send(["agent_alive:feedback", int(mess[1]) in env.agent_ids])
        elif mess[0] == "render:save":
            env.save_render()
            conn.send(["render:save:feedback"])
        elif mess[0] == "close:call":
            env.close()
            conn.send(["close:feedback"])


class MultiAgentClient(MultiAgentEnv):
    def __init__(self, observation_space, action_space, agents, env_setup,
                 scenario, max_step, render=False):
        self.observation_space = observation_space
        self.action_space = action_space
        self._agents = agents
        self._render = render
        self._max_step = max_step

        global client_id

        server, self.client = Pipe()
        self.p = Process(target=engine_handler,
                         args=(server, env_setup, scenario), name='group-{}-{}'.format(client_id, int(time.time())))
        client_id += 1
        self.p.start()

        self._step = 0

        if render:
            self.client.send(["render:on"])
            mess = self.client.recv()
            assert mess[0] == "render:on:feedback"

    def reset(self):
        self.client.send(["reset:call", self._render])
        mess = self.client.recv()

        assert mess[0] == "reset:feedback"
        
        state_n = dict(zip(
            map(str, mess[1].keys()),
            mess[1].values()
        ))
        self._step = 0
        return state_n

    def step(self, action_dict):
        self._step += 1
        self.client.send(["step:action", action_dict])
        mess = self.client.recv()

        assert mess[0] == "step:feedback"

        # convert int key to str
        next_state_n, reward_n, done_n = mess[1], mess[2], mess[3]
        next_state_n = dict(
            zip(map(str, next_state_n.keys()), next_state_n.values()))
        reward_n = dict(zip(map(str, reward_n.keys()), reward_n.values()))
        done_n = dict(zip(map(str, done_n.keys()), done_n.values()))

        done_n["__all__"] = np.alltrue(list(done_n.values()))

        if (done_n['__all__'] or self._step >= self._max_step) and self._render:
            self.client.send(["render:save"])
            mess = self.client.recv()
            assert mess[0] == "render:save:feedback"

        return next_state_n, reward_n, done_n, {}

    def close(self):
        self.p.terminate()

    def is_dead(self, agent_id):
        self.client.send(["agent_alive:check", agent_id])

        mess = self.client.recv()

        assert mess[0] == "agent_alive:feedback"

        return mess[1]