import sys
import os
import dill as pickle
import os.path as osp
import argparse
import pprint
import ray

from ray.rllib.agents import ppo, pg
from ray.tune.registry import register_env
from ray.rllib.agents.registry import get_agent_class
from ray.rllib.rollout import rollout
from ray.rllib.agents.trainer import COMMON_CONFIG

from pyds import engine
from pyds import scenarios
from pyds import utils
from pyds import multiprocess_wrapper


BASE_DIR = osp.dirname(osp.dirname(osp.abspath(__file__)))
CONFIG_BACKUP = osp.join(BASE_DIR, "configs")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--max_step",
                        type=int,
                        default=400,
                        help="Max step per episode (default is 400)")
    parser.add_argument("--config",
                        type=str,
                        required=True,
                        help="Configuration filename for MARl algorithms.")
    parser.add_argument("--checkpoint",
                        type=str,
                        required=True,
                        help="Checkpoint for model loading")
    parser.add_argument("--out", type=str, default="out.pkl")
    args = parser.parse_args()

    with open(args.config, 'rb') as f:
        configs = pickle.load(f)

    ray.init()
    env_config = configs['ray_config']['env_config']  # ['env_name']
    env_config['render'] = True  # turn on render mode
    env_name = env_config['env_setup']['env_name']

    register_env(
        env_name, lambda kwargs: multiprocess_wrapper.MultiAgentClient(
            **kwargs))

    cls = get_agent_class(configs["run"])
    agents = cls(env=env_name, config=configs['ray_config'])
    agents.restore(args.checkpoint)

    rollout(agents, env_name, args.max_step, args.out)
    agents.stop()
