import importlib as imp
import os.path as osp

from . import simple, highway

SimpleScenario = simple.Scenario
HighWayScenario = highway.Scenario


__all__ = ["simple", "highway"]


def load(name):
    if name == "simple":
        return SimpleScenario
    elif name == "highway":
        return HighWayScenario
    else:
        raise NotImplementedError
