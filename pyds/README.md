## Python Package for DS

### Structure

- `engine.py`:
- `scenarios/base.py`:

**Customize Your Scenario**

You can customize your scenario with inheriting from `scenarios/baser.py::BaseScenario`. A simple example is `scenarios/simple.py`. 
Methods which must be implemented are listed below:

- `observation_space`: specify the observation shape with a 3-dim tuple like (`channel`: 3, `width`: 50, `height`: 50). NOTE: channel is fixed currently, do not change it
- `action_space`: specify the action shape with a tuple like (3,). If you use the built-in action space, you can retrieve a dict of agent action space with calling `action_space` of environment
- `reset_world`: reset engine
- `reward`:
- `observation`: agent-wise get observation
- `done`: agent-wise get terminate agent state


### Usage

1. load environment set up from local json file and scenario component
2. policy configuration
3. register environment using ray
4. init trainer
5. run
