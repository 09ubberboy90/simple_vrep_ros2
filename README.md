# simple_vrep_ros2

This repository contains 2 tasks for Coppeliasim  

## Stacking Task
![Alt text](https://github.com/09ubberboy90/simple_ignition_ros2/blob/a59ab173732613deca22c6ee2fb0c64cf43ecc09/imgs/Ignition_place.png "Stacking Cube task")
This task involves one Franka Panda robotic arm randomly taking 5cm cubes from a table with 21 cubes arranged in a 7 * 3 grid and stacking them. It will stack them into 3 stacks of 5 cubes. The number of columns and the the heigt can be configured in [moveit_controller.launch.py](https://github.com/09ubberboy90/simple_ignition_ros2/blob/a59ab173732613deca22c6ee2fb0c64cf43ecc09/simple_arm/launch/moveit_controller.launch.py#L63)

## Throw Task
![Alt text](https://github.com/09ubberboy90/simple_ignition_ros2/blob/a59ab173732613deca22c6ee2fb0c64cf43ecc09/imgs/Ignition_throw.png "Throw Task ")

For this, a Franka Panda arm will pick up a cube and throw it toward a pyramid made of 6 cubes. The arm will go as far back as it can and perform a throwing motion toward the ground in front of it. The ball will be released at 50% of the trajectory as it is the moment with the most amount of forces. The pyramid is placed such that a successful throw at full power will collapse it. However in Ignition the cube drops when the arm is as far back as possible

## Instructions

### Requirements

- **OS:** Ubuntu 20.04 (Focal)
  - Other distributions might work (not tested).

### Dependencies

These are the primary dependencies required to use this project.

- ROS 2 [Foxy](https://docs.ros.org/en/foxy/Installation.html)
- Coppelisim [v4.2.0](https://www.coppeliarobotics.com/)
- [MoveIt 2](https://moveit.ros.org/install-moveit2/binary)
  - Install/build a version based on the selected ROS 2 release

Furthermore, the following packages are required.

- [ros_ign](https://github.com/ignitionrobotics/ros_ign/tree/ros2)
  - Install/build a version based on the selected combination of ROS 2 release and Ignition version
- [sim_ros2_interface](https://github.com/CoppeliaRobotics/simExtROS2.git)
  - A working version has been added as a submodule. Newer version should work but are not tested. 

### Building

Clone this repository and import submodules. Then install dependencies and build with [colcon](https://colcon.readthedocs.io).

```bash
# Clone this repository and the submodules into your favourite ROS 2 workspace
git clone --recurse-submodules https://github.com/09ubberboy90/simple_vrep_ros2.git
# Install external dependencies via rosdep
rosdep install -r --from-paths src --ignore-src --rosdistro ${ROS_DISTRO}
# Build with colcon
colcon build --merge-install --symlink-install --cmake-args "-DCMAKE_BUILD_TYPE=Release"
```

### Sourcing

Before utilising this package, remember to source the ROS 2 workspace overlay.

```bash
source ${ROS2_WS}/install/local_setup.bash
export COPPELIASIM_ROOT_DIR ${Path}/CoppeliaSim_Edu_V4_2_0_Ubuntu20_04/
export COPPELIASIM_ROOT ${Path}/CoppeliaSim_Edu_V4_2_0_Ubuntu20_04/

```

## Running

The following tasks can be either run as an independant launch or can be recorded

### Stacking cube

```
ros2 launch panda_vrep stack_cubes.launch.py
```
### Throwing cube

```
ros2 launch panda_vrep throw_cubes.launch.py
```
## Recorder
The recorder can be launched with. 
```
ros2 run sim_recorder run_recording 
```
It has some argument option to configure what the task will be
```
--help: Gives a list of arguments
-t, --throw: Start the Throwing Task
-i, --iteration: Number of run that should be performed
-s, --start: Start index in case some run need to be rerun after an interuption
--headless: runs without a GUI
```

The recorder will save the CPU and RAM usage in `sim_recoder/data/{sim_name}`.
`sim_name` is created automatically based ont the simulation name `vrep` and on the arguments:
- `_gui` is added if `--headless` is not passed
- `_throw` is added if `--throw` is passed

An analysis of the run can be performed by running from the source of the repo
```
python ./sim_recorder/sim_recoder/grapher.py {name_of_folder}
```
With name of folder being the name of the folder you want to analyse
