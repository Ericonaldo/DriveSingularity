# DriveSingularity
A scalable multi-agent learning simulator for autonomous driving

## Installation

**Step 1: install the requirements**

For Ubuntu,
```shell script
[sudo] apt install git build-essential cmake python3 python3-pip libjsoncpp-dev
pip3 install -r requirements.txt
```

For Mac OS X,
```shell script
brew install cmake python3 jsoncpp
pip3 install -r requirements.txt
```

**Step 2: Clone the project, build and install**

```shell script
git clone --recursive https://github.com/KornbergFresnel/DriveSingularity.git
cd DriveSingularity
mkdir build
cd build
cmake ..
make
cd ..
```
**Step 3: install `pyds`**

```shell script
pip3 install .
```

## Usage

### Load scenario

DriveSingularity supports custom scenario registration, so you need to load configuration from local scenario configuration file before generating environment. There is a default scenario configuration file locates at `DriveSingularity/configs/simple_env_config.json`.

```python
import os.path as osp
from pyds import utils

BASE_DIR = ${WORKSPACE}/
CONFIG_BACKUP = osp.join(BASE_DIR, "configs")

env_config = utils.load_env_config(CONFIG_BACKUP, "simple_env_config.json")
```

For more details on scenario configuration, please refer to [Scenario configuration](#scenario-configuration)


### Generate environment

Environment generation requires only one command:

```python
from pyds import scenarios

env = engine.Environment(env_config,
                             scenario=scenarios.load(args.scenario)())
```

Then, you can run it as Gym environments:

```python
import numpy as np

done = False
state_dict = env.reset()

while not done:
    action_dict = ...
    next_state_dict, reward_dict, done_dict, _ = env.step(action_dict)
    done = np.alltrue(list(done_dict.values()))
    # ...
```

### Learning with RLLib

The default training support is [RLLib](https://ray.readthedocs.io/en/latest/rllib.html). To enable the interaction between engine and the training toolkit, we offer a multiprocess environment wrapper: `DriveSingularity/pyds/multiprocess_wrapper.py`, you can get more details from the training example: `DriveSingularity/examples/multiprocess_train.py`

### Render: execute rollout.py

After you have completed the training, you can execute the `examples/rollout.py` to get the render files, then you can use [DSRender](https://github.com/kornbergfresnel/DSRender) to replay the render files.

## Scenario configuration

A simple example:

```json
{
    "world": "config/resoureces/osm.json",
    "env_name": "simple_osm",
    "render_dir": "render/osm",
    "n_agents": 2,
    "total_vehicles": 40,
    "length_range": [15, 1],
    "width_range": [8, 1],
    "height_range": [15, 3],
    "velocity_range": [0, 5],
    "target_velocity_range": [10, 10],
    "halfLengthRange": [12, 14],
    "halfWidthRange": [7, 8],
    "velocityRange": [10, 20],
    "targetVelocityRange": [30, 50]
}
```

**Environment macro settings**

- `world`: relative path of road map, must be json format.
- `env_name`: environment name for `registery.register_env`.
- `render_dir`: specify the relative path for the rendering result storage.
- `n_agents`: specify the maximum number of agents.
- `total_vehicles`: specify the maximum number of **agents + social vehicles**.

**Configuration for agent vehicles**

- `length_range`: for agent, specify the length of agent vehicle, **left value** denotes the minimum, **right value** denotes the range, i.e, the range of length will be: `[left vaue, left value + right value]`.
- `width_range`: for agent, specify the width of agent vehicle, **left value** denotes the minimum, **right value** denotes the range, i.e, the range of width will be: `[left vaue, left value + right value]`.
- `height_range`: useless (currently).
- `velocity_range`: for agent, specify the initialized speed, **left value** denotes the initialized speed, **left value** denotes the minimum, **right value** denotes the range, i.e, the range of initialized speed will be: `[left vaue, left value + right value]`.
- `target_velocity_range`: for agent, specify the target speed if no extra interruption (i.e. agent actions), **left value** denotes the minimum, **right value** denotes the range, i.e, the range of target speed will be: `[left vaue, left value + right value]`.

**Configuration for social vehicles**

- `halfLengthRange`: for social vehicles, specify the half length, with the given range, the length of social vehicle will be `[left value * 2, right value * 2]`.
- `halfWidthRange`: for social vehicles, specify the half width, with the given range, the width of social vehicle will be `[left value * 2, right value * 2]`.
- `velocityRange`: for social vehicles, specify the start speed, with the given range, the initialized speed of social vehicle will be `[left value * 2, right value * 2]`.
- `targetVelocityRange`: for social vehicles, specify the target speed, with the given range, the target speed of social vehicle will be `[left value * 2, right value * 2]`