from setuptools import setup, find_packages

setup(
    name="ds",
    version="0.1",
    package_dir={'ds': 'python'},
    packages=find_packages(),
    package_data={
        '': ['*.so']
    }
)
