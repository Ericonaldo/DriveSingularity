# DriveSingularity
A scalable multi-agent learning simulator for autonomous driving

## Installation

**Step 1: install the requirements**

```shell script
[sudo] apt install git build-essential cmake python3 python3-pip libjsoncpp-dev
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
