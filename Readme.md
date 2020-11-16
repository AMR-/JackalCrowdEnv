
NOTE: THIS REPO IS slightly under construction

This repository contains <!-- a few things -->

* JackalCrowdEnv, an OpenAI gym style environment for running navigation tasks with the Jackal robot in Gazebo and ROS
* Documentation for the above, including explanation of various scenarios used
<!-- * Code to generate trajectories and run the VIPER or MSVIPER procedures, as noted in a pending conference paper (to be added upon publication) -->


# Installation and Setup

System requirements:
```
    Ubuntu: 18.04
    ROS: melodic
    python: 3.6
    Anaconda
```
Install ROS Jackal packages:
```
	sudo apt-get install ros-melodic-jackal*
```

If you have no catkin workspace yet, create one:
```
   mkdir -p ~/catkin_ws/src
   cd catkin_ws/src
   echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

If you already have one, then navigate to its `src` folder.

Either way, continue below:

```
   git clone https://github.com/AMR-/JackalCrowdEnv.git
   cd ..
   catkin_make
   source devel/setup.bash
```

To get the packages in virtual environment:

```
pip install -f packages.txt
```

# Usage

Note: the ROS name for the package is `naviswarm` 

### Bring up the simulator
In one terminal:
```
    roslaunch naviswarm jackal_world.launch
```

The above command must be run ahead of instantiating the CrowdEnv object.

### Import CrowdEnv

Import CrowdEnv
```
from crowdenv.rl import CrowdENV
```
Instatiate CrowdEnv, here are some examples:

```
env = CrowdENV()

env = CrowdENV(scenarios_index=10, 
               max_steps=1000, 
               random_seed=0)
```

Here is an explanation of each of the arguments of CrowdEnv:

* __scenarios_index__: _int_  - indicating the id of the environmental scenario (obstacle and goal configuration) to set up in gazebo. Default is the empty scenario. See the Scenarios Section Below
* __collision_threshold__: _float_ - how close (in meters) robot must be to object for it to be considered a collision
* __target_threshold__: _float_ - how close (in meters) robot must be to goal for it to be considered a goal
* __step_time__: _float_ - how many seconds per timestep (default 0.1)
* __max_steps__: _int_ - maximum number of timesteps per episode even if goal is not reached and collisio does not occur
* __random_seed__: _int_ - there is some randomness used in the environment, use this to set the random seed
* __vel_expanded__: _bool_ - set to False (default) to use the 6 action action space, or True to use the expanded 10 action action space. See details in Action Spaces below

## Scenarios

| # | Description |
| :--- | :--- |
| 0 | Empty env. where goal is 2m up right side of robot  |
| 1 | Empty env. where goal is 2m down right side of robot |
| 2 | Empty env. where goal is 2m ahead robot |
| 3 | Empty env. where goal is 2m on back of robot |
| 4 | Empty env. where goal is 10m up right side of robot |
| 5 | Empty env. where goal is 10m down right side of robot |
| 6 | Empty env. where goal is 10m ahead robot |
| 7 | Empty env. where goal is 10m on back of robot |
| 8 | Empty env. in the down side area where goal and start locations are all random |
| 9 | Empty env. in the up side area where goal and start locations are all random |
| 10 | Cross shape obstacle in middle of the area, robot starts from left side of the obstacle |
| 11 | Cross shape obstacle in middle of the area, robot starts from right side of the obstacle |
| 12 | Diomand shape obstacle in middle of the area, robot starts from left side of the obstacle |
| 13 | Diomand shape obstacle in middle of the area, robot starts from right side of the obstacle |


## Action Spaces

Standard Action Space:

| # | Description | Linear Vel. | Angular Vel |
| :--- | :--- | :--- | :--- |
| 0 | Forward Right | 1 | -1 |
| 1 | Rotate Right | 0 | -1 |
| 2 | Straight Forward | 1 | 0 |
| 3 | Stop | 0 | 0 |
| 4 | Forward Left | 1|1 |
| 5 | Rotate Left |0| 1|

Expanded Action Space:

| # | Description | Linear Vel. | Angular Vel |
| :--- | :--- | :--- | :--- |
| 0-5 | (Same as standard) | | |
| 6 | Slightly Forward Right | 0.5 | -0.5 |
| 7 | Slightly Rotate Right | 0 | -0.5 |
| 8 | Slightly Forward Left | 0.5|0.5 |
| 9 | Slightly Rotate Left |0| 0.5|

## State Space
content: | goal | velocity | Occupancy Grid of Lidar |

size:    |   2  |     1    |          210            |
<!-- goal, past, and polar grid with picture -->
