import numpy as np
import gym
import sys
import os
import os.path as osp
import traceback
from gym import spaces
from . import simulator

BASEDIR = osp.dirname(osp.dirname(osp.abspath(__file__)))

# sys.path.append(osp.join(BASEDIR, "cmake-build-debug"))


def parse_feedback(feedback_list, reward_parser):
    id_n, obs_n, done_n, reward_n = [], dict(), dict(), dict()

    for feedback in feedback_list:
        id_n.append(feedback.id)
        obs_n[feedback.id] = np.swapaxes(
            np.asarray(feedback.observation, dtype=np.float32), 0, 2)
        done_n[feedback.id] = feedback.done
        reward_n[feedback.id] = reward_parser(feedback.id, feedback.events)

    return obs_n, reward_n, done_n, id_n


class Environment(gym.Env):
    metadata = {"render.modes": ["human", "rgb_array"]}

    def __init__(self, env_setup: dict, scenario):
        """ Initialize a gym environment wrapper
        """
        print("[DEBUG] engine.py::__init__ start road map builder")
        builder = simulator.RoadMapBuilder()
        builder.load(osp.join(BASEDIR, env_setup["world"]))
        builder.build()

        road_map = builder.get_road_map()
        road_graph = builder.get_road_graph()

        # print("[DEBUG] engine.py::__init__ start environment builder")
        # TODO(yifan): self._env = simulator.Environment(road_map, road_graph, action="discrete")
        self._env = simulator.Environment(road_map, road_graph)
        print('[DEBUG] enginey.py::__init__ start add agents.')
        self._env.load_vehicles(env_setup["agents"])
        print("[DEBUG] engine.py::__init__ set vehicle generator")
        self._env.set_social_vehicle_generator(
            env_setup["total_vehicles"],
            env_setup["halfLengthRange"],
            env_setup["halfWidthRange"],
            env_setup["velocityRange"],
            env_setup["targetVelocityRange"]
        )

        # TODO(ming): initialize render and agents
        self._env.set_render(env_setup['render_dir'])

        # sys.settrace(self._env.step)

        # TODO(ming): create generators for social vehicles
        # self._env.load_generator(env_setup["generators"])
        self._env_setup = env_setup

        agent_ids = self._env.agents()
        print(agent_ids)
        assert len(agent_ids) > 0  # checked

        self._scenario_callback = scenario
        self._scenario_callback.init(env=self._env)

        self._scenario_callback.register_reward_rule(self._env)
        self._render = False

    @property
    def agent_ids(self):
        return self._env.agents()

    @property
    def observation_space(self):
        return self._scenario_callback.observation_space  # dict: key[agent_id], value[gym.space]

    @property
    def action_space(self):
        return self._scenario_callback.action_space  # dict: key[agent_id], value[gym.space]

    def step(self, action: dict):

        # try:
        feedback = self._env.step(action)
        obs_n, reward_n, done_n, info_n = parse_feedback(
            feedback, self._get_reward)
        return obs_n, reward_n, done_n, info_n
        # except Exception as e:
        #     print("[DEBUG] >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")
        #     traceback.print_exc()

    def reset(self, episode=None):
        self._env.reset()

        sub_dir = osp.join(self._env_setup['render_dir'], str(episode))
        if not osp.exists(sub_dir) and self._render:
            os.makedirs(sub_dir)
    
        self._env.set_render(sub_dir)
        self._env.load_vehicles(self._env_setup["agents"])
        assert len(self._env.agents()) > 0
        self._scenario_callback.register_reward_rule(self._env)

        agent_ids = self._env.agents()

        assert len(agent_ids) > 0

        self._scenario_callback.init(env=self._env)

        self._scenario_callback.register_reward_rule(self._env)

        # check consistency
        # print(self._scenario_callback.observation_space.keys(), agent_ids)
        assert list(self._scenario_callback.observation_space.keys()) == list(
            agent_ids)

        obs_n = dict.fromkeys(agent_ids)

        for key in obs_n:
            obs_n[key] = self._scenario_callback.observation(key, self._env)

        return obs_n

    def turn_on_render(self):
        self._env.turn_on_render()

    def turn_off_render(self):
        self._env.turn_off_render()

    def render(self, mode='human'):
        if self._render:
            self._env.render()

    def save_render(self):
        self._env.save_render()

    def close(self):
        self._env.reset()
        del self._env
        self._env = None

    def _get_obs(self, agent):
        return self._scenario_callback.observation(agent, self._env)

    def _get_done(self, agent):
        return self._scenario_callback.done(agent, self._env)

    def _get_reward(self, agent, events):
        return self._scenario_callback.reward(agent, events)

    def _set_action(self, action, agent_id, action_space, time=None):
        raise NotImplementedError

    def _reset_render(self):
        raise NotImplementedError
