# AUAV 2022 Sample

This is a sample PX4 based quadrotor path planning framework based on Ubuntu 20.04 and ROS noetic for the IEEE Autonomous UAS 2022 competition.

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

Run the simulation for each trial.

**Note: To fly PX4 you will need to have QGroundControl running and manually arm it and put it in
offboard mode. For the competition we do not want your scirpts to do this automatically. For testing and in simulation, you can have a script that manually arms the drone and puts it in offboard mode.**

We are following this safety procedure so that the pilot will have the only 
authority to arm the drone and switch the mode to offboard from the RC transmitter.

```bash
cd ~/catkin
. ./devel/setup.bash
roslaunch auav_2022_sample sim.launch world:=worlds/trial_1.world 
roslaunch auav_2022_sample sim.launch world:=worlds/trial_2.world 
roslaunch auav_2022_sample sim.launch world:=worlds/trial_3.world 
roslaunch auav_2022_sample sim.launch world:=worlds/trial_4.world 
```

Please adapt your code to work with this framework, we expect to launch
each drone with the command:
```bash
roslaunch auav_2022_sample live.launch
```

## Path Planning

Two examples of path planning libraries are included that use the depth camera information:
* PX4 avoidance: Uses VFH3D+ approach, simple approach, not very robust
* path planning: Uses octomap and OMPL library for (RRT), could be improved to use Quadrotor dynamics
