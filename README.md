# MRS UAV Gazebo Simulation

![](.fig/thumbnail.jpg)

A metapackage for the MRS UAV Gazebo + PX4 SITL simulation pipeline.

## Installation

1. Install the [MRS UAV System](https://github.com/ctu-mrs/mrs_uav_system#installation)
2. `sudo apt install ros-noetic-mrs-uav-gazebo-simulation`

## Submodules

| Package                     | Repository                                                                            |
|-----------------------------|---------------------------------------------------------------------------------------|
| MRS Gazebo Common Resources | [mrs_gazebo_common_resources](https://github.com/ctu-mrs/mrs_gazebo_common_resources) |
| PX4                         | [px4_firmware](https://github.com/ctu-mrs/px4_firmware)                               |
| PX4 SITL Gazebo             | [px4_sitl_gazebo](https://github.com/ctu-mrs/px4_sitl_gazebo)                         |

## Unmanned Aerial Vehicles

### Pre-defined UAVs

| Model         | Spawn argument | Simulation                    |
|---------------|----------------|-------------------------------|
| DJI f330      | `--f330`       | ![](.fig/f330_simulation.jpg) |
| DJI f450      | `--f450`       | ![](.fig/f450_simulation.jpg) |
| Holybro x500  | `--x500`       | ![](.fig/x500_simulation.jpg) |
| DJI f550      | `--f550`       | ![](.fig/f550_simulation.jpg) |
| Tarot t650    | `--t650`       | ![](.fig/t650_simulation.jpg) |
| T-Drones m690 | `--m690`       | ![](.fig/m690_simulation.jpg) |
| NAKI II       | `--naki`       | ![](.fig/naki_simulation.jpg) |

### Adding a custom UAV

A custom drone model can be added from an external package.
Please look at [mrs_gazebo_custom_drone_example](https://github.com/ctu-mrs/mrs_gazebo_custom_drone_example) for an example.
The wiki page [https://ctu-mrs.github.io/docs/simulation/gazebo/gazebo/custom_drone.html](https://ctu-mrs.github.io/docs/simulation/gazebo/gazebo/custom_drone.html) contains a detailed description of all the important steps and configuration parts.

## Starting the simulation

Use one of the prepared Tmuxinator sessions in [`roscd mrs_uav_gazebo_simulation/tmux`](./ros_packages/mrs_uav_gazebo_simulation/tmux) as an example:

- [one_drone](./ros_packages/mrs_uav_gazebo_simulation/tmux/one_drone)
- [one_drone_3dlidar](./ros_packages/mrs_uav_gazebo_simulation/tmux/one_drone_3dlidar)
- [one_drone_realsense](./ros_packages/mrs_uav_gazebo_simulation/tmux/one_drone_realsense)
- [three_drones](./ros_packages/mrs_uav_gazebo_simulation/tmux/three_drones)

## Using the MRS drone spawner in your simulations

The drone models are dynamically created in runtime using the [MRS drone spawner](https://ctu-mrs.github.io/docs/simulation/gazebo/gazebo/drone_spawner.html). The UAV platforms can be additionally equipped by adding [components](ros_packages/mrs_uav_gazebo_simulation/models/mrs_robots_description/sdf/component_snippets.sdf.jinja) (rangefinders, LiDARs, cameras, plugins etc.).

### Start the Gazebo simulator

To start the example Gazebo world call:

```bash
roslaunch mrs_uav_gazebo_simulation simulation.launch world_name:=grass_plane.world gui:=true
```

At this point the Gazebo world will only contain the environment with grass plane but with no vehicles yet.

### Spawning the UAVs

The `simulation.launch` will automatically start the `mrs_drone_spawner` as a ROS node. If you use a custom launch file to start Gazbo, you can launch the spawner separately:

```bash
roslaunch mrs_uav_gazebo_simulation mrs_drone_spawner.launch
```

The `mrs_drone_spawner` will perform the following tasks:

* Generate SDF models from the UAV templates

* Add optional components (sensors, plugins...) based on the user input

* Run PX4 SITL and Mavros, and ensure that all ports are correctly linked with the Gazebo simulator

* Remove all subprocesses on exit

Vehicles are added to the simulation by calling the `spawn` service of the `mrs_drone_spawner`.
The service takes one string argument, which specifies the vehicle ID, type and sensor configuration.
Example: spawn a single vehicle with ID 1, type X500, with a down-facing laser rangefinder:

```bash
rosservice call /mrs_drone_spawner/spawn "1 --x500 --enable-rangefinder"
```

To display the basic use manual for the spawner, call the service with the argument ` --help`. **NOTE**: String argument cannot start with a dash. Add a space before the dashes to avoid errors. The service call returns the full help text, but the formatting may be broken. Please refer to the terminal running `simulation` or `mrs_drone_spawner` where the help text is also printed with proper formatting.

```bash
rosservice call /mrs_drone_spawner/spawn " --help"
```

You can also display a manual for a specific platform. This will list all the components that can be equipped to the selected platform, and their brief description.
```bash
rosservice call /mrs_drone_spawner/spawn " --x500 --help"
```

Multiple vehicles may be spawned with one service call:
```bash
rosservice call /mrs_drone_spawner/spawn "1 2 3 4 5 --t650 --enable-bluefox-camera --enable-rangefinder"
```

The default parameters of some components may be reconfigured by adding `param:=value` after the component keyword. Multiple params may be used at the same time:
```bash
rosservice call /mrs_drone_spawner/spawn "1 --x500 --enable-rangefinder --enable-ouster model:=OS0-32 use_gpu:=True horizontal_samples:=128 update_rate:=10"
```
The list of components and their reconfigurable parameters can be displayed using the platform-specific help.

For more details, please refer to the [MRS drone spawner](https://ctu-mrs.github.io/docs/simulation/gazebo/gazebo/drone_spawner.html) page.
