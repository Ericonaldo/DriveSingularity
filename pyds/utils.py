import json
import random
import os.path as osp


def generate_vehicle(n_agents, length_range, width_range, height_range,
                     velocity_range, target_velocity_range, vehicle_type):

    agents = []

    for _ in range(n_agents):
        agents.append({
            "halfLength":
            random.random() * length_range[1] + length_range[0],
            "halfWidth":
            random.random() * width_range[1] + width_range[0],
            "halfHeight":
            random.random() * height_range[1] + height_range[0],
            "velocity":
            random.random() * velocity_range[1] + velocity_range[0],
            "targetVelocity":
            random.random() * target_velocity_range[1] + target_velocity_range[0],
            "type": vehicle_type
        })

    return agents


def write_vehicle_config(base_dir, n_agent, render_dir):
    vehicles = []

    x, y = [100, 200, 300], [120, 220, 260, 300]

    for x_i in x:
        for y_i in y:
            v = random.random() % 100 + 40
            vehicles.append({
                "x": x_i,
                "y": y_i,
                "halfLength": 15,
                "halfWidth": 8,
                "halfHeight": 15,
                "rotation": 0,
                "velocity": 40,
                "maxVelocity": v,
                "type": 0
            })

            n_agent -= 1

            if n_agent == 0:
                break

        if n_agent == 0:
            break

    configuration = {
        "world": osp.join(base_dir, "resources/onramp2.json"),
        "vehicles": vehicles,
        "render_dir": render_dir
    }

    if not osp.exists(osp.join(base_dir, "v_onramp2.json")):
        with open(osp.join(base_dir, 'v_onramp2.json'), 'w') as f:
            json.dump(configuration, f)

    return configuration


def _retrieve_lanes(roads: dict):
    res = []

    for road in roads:
        tmp = list(zip([road["id"]] * len(road["lanes"]), range(len(road["lanes"]))))
        res.extend(tmp)

    return res
    

def load_env_config(config_dir, config_path):
    with open(osp.join(config_dir, config_path), 'r') as f:
        configuration = json.load(f)

    configuration["agents"] = generate_vehicle(
        configuration["n_agents"], configuration["length_range"], configuration["width_range"],
        configuration["height_range"], configuration["velocity_range"],
        configuration["target_velocity_range"], vehicle_type=0)

    return configuration


def policy_mapping_fn(key, extra=None):
    if extra is not None:
        for_share = random.choice(list(extra))
    else:
        for_share = None

    return {
        "one_to_one": lambda agent_id: agent_id,
        "random": lambda agent_id: random.choice(extra["agent_ids"]),
        "share": lambda agent_id: for_share
    }[key]
