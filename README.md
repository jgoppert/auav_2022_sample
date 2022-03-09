# AUAV 2022 Sample

This is a demonstration of how to use PX4 avoidance on Ubuntu 20.04 and ROS noetic for the IEEE Autonomous UAS 2022 competition.

You will need to install xterm and the standard PX4/gazebo requirements.
```bash
sudo apt install xterm
```

Build the catkin workspace.
```bash
mkidr ~/catkin
cd ~/catkin
mkdir src
cd src
git clone https://github.com/jgoppert/auav_2022_sample.git
git submodule update --init --recursive
catkin build
```

Run the simulation for each trail.
```bash
cd ~/catkin
. ./devel/setup.bash
roslaunch auav_2022_sample world:worlds/trial_1.world sim.launch
roslaunch auav_2022_sample world:worlds/trial_2.world sim.launch
roslaunch auav_2022_sample world:worlds/trial_3.world sim.launch
roslaunch auav_2022_sample world:worlds/trial_4.world sim.launch
```

Please adapt your code to work with this framework, we expect to launch
each drone with the command:
```bash
roslaunch auav_2022_sample live.launch
```

T
