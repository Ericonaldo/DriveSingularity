import sys
import os
import dill as pickle
import os.path as osp
import argparse
import pprint
import ray
import time

from ray.rllib.agents import ppo, pg
from ray.tune.registry import register_env
from ray.rllib.agents.registry import get_agent_class, ALGORITHMS
from ray.rllib.agents.trainer import COMMON_CONFIG

from pyds import engine
from pyds import scenarios
from pyds import utils
from pyds import multiprocess_wrapper

BASE_DIR = osp.dirname(osp.dirname(osp.abspath(__file__)))
CONFIG_BACKUP = osp.join(BASE_DIR, "configs")
MODEL_BACKUP = osp.join(BASE_DIR, "model")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--max_step",
                        type=int,
                        default=400,
                        help="Max step setting.")
    parser.add_argument("--scenario",
                        type=str,
                        default="simple",
                        choices=scenarios.__all__,
                        help="Scenario selection, default is `simple`")
    parser.add_argument("--n_epoch",
                        type=int,
                        default=100,
                        help="Learning episodes.")
    parser.add_argument("--interval", type=int, default=20, help="Save interval, defaut is 20.")
    parser.add_argument("--run", type=str, default="PG", choices=set(ALGORITHMS.keys()))
    parser.add_argument("--share", action="store_true")
    parser.add_argument("--n_workers", type=int, default=2)
    parser.add_argument("--n_gpus", type=float, default=1.)
    parser.add_argument("--n_cpu_per_worker", type=float, default=0.)
    parser.add_argument("--s_batch_size", type=int, default=200)
    parser.add_argument("--t_batch_size", type=int, default=200)
    parser.add_argument("--config", type=str, default="simple_env_config.json", help="Environment configuration filename.")

    args = parser.parse_args()

    env_config = utils.load_env_config(CONFIG_BACKUP, args.config)
    env = engine.Environment(env_config,
                             scenario=scenarios.load(args.scenario)())
    agent_ids, action_space, observation_space = env.agent_ids, env.action_space, env.observation_space
    assert len(agent_ids) > 0

    env.close()

    pre = [None] * len(agent_ids)
    hyper_parameters = [{"gamma": 0.95}] * len(agent_ids)

    # policy_configs: for non-parameter sharing mode, i.e. one policy per agent
    policy_configs = zip(pre, env.observation_space.values(),
                         env.action_space.values(), hyper_parameters)
    policy_configs = dict(zip(map(str, agent_ids), policy_configs))

    ray.init()

    register_env(env_config['env_name'], lambda kwargs: multiprocess_wrapper.MultiAgentClient(**kwargs))

    ray_config = {
        "multiagent": {
            "policies": {} if args.share else policy_configs,
            "policy_mapping_fn": None if args.share else utils.policy_mapping_fn("one_to_one"),
            "policies_to_train":
            None  # optional whitelist of policies to train, or None for all policies
        },
        "no_done_at_end": False,
        "horizon": args.max_step,
        "num_workers": args.n_workers,  # number of actors used for parallelism
        "num_gpus":
        args.n_gpus,  # can be fractional, e.g., 1 GPU for 5 agent is: 0.2
        "num_cpus_per_worker": args.n_cpu_per_worker,
        "sample_batch_size": args.s_batch_size,
        "train_batch_size": args.t_batch_size,
        "env_config": {
            "observation_space": observation_space,
            "action_space": action_space,
            "agents": agent_ids,
            "env_setup": env_config,
            "scenario": args.scenario,
            "max_step": args.max_step,
            "render": False
        }
    }

    trainer = get_agent_class(args.run)(env=env_config['env_name'], config=ray_config)

    # save current configuration
    config_path = osp.join(CONFIG_BACKUP,
                           "{}_{}_{}_{}.pkl".format(args.scenario, args.run, len(agent_ids), "share" if args.share else "non_share"))
    if not osp.exists(CONFIG_BACKUP):
        os.makedirs(CONFIG_BACKUP)

    # agent model backup
    timestamp = time.time()
    model_backup = osp.join(
        MODEL_BACKUP, "{}_{}_{}_{}_{}".format(args.scenario, args.run, len(agent_ids),
                                              "share" if args.share else "non_share", timestamp))

    with open(config_path, 'wb') as f:
        pickle.dump(
            {
                "ray_config": ray_config,
                "run": args.run
            }, f)

    for epoch in range(args.n_epoch):
        print("[INFO] training epoch: {}".format(epoch))

        trainer.train()
        if (epoch + 1) % args.interval == 0:
            trainer.save(checkpoint_dir=model_backup)

    print("[training finished]")
    trainer.stop()
