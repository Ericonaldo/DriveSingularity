from .random import RandomPolicy

random = random.RandomPolicy

__all__ = ['random']


def load(name):
    lib = {
        "random": random
    }

    res = None

    if isinstance(name, str):
        res = lib[name]
    elif isinstance(name, list):
        res = []
        for e in name:
            res.append(lib[e])
    else:
        raise NotImplementedError

    return res
