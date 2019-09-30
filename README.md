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
**Step 3: install `pyds` and requirements for training**

```shell script
pip3 install .
pip3 install -r requirements.txt
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


