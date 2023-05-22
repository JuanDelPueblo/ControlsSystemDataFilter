# Controls System Data Filter

![image](https://github.com/JuanDelPueblo/ControlsSystemDataFilter/assets/49998039/66377ef6-fd11-4c4c-be4c-89fa7e9651ca)

Merges the velocity, orientation, position, and pressure data from the IMU, DVL, and pressure sensor into a buffer message that is then published at a continuous rate.

Tested on Ubuntu 18.04.6 LTS with ROS Melodic Morenia, and depends on the [uuv_simulator](https://github.com/uuvsimulator/uuv_simulator) and [rexrov2](https://github.com/uuvsimulator/rexrov2) packages

Project built as a part of training for RUMarino.

### Usage

To run the project, simply clone this repository into the Catkin workspace `src` folder containing both the uuv_simulator and rexrov2 packages. Then run the following commands in order while inside the workspace root directory:

```
catkin_make
source devel/setup.bash
roslaunch controls_system_data_filter cs.launch
```

This will launch the ocean_waves environment in Gazebo, spawn a RexROV 2 UUV in the simulation, and finally start the controls system data filter, which will begin publishing the buffer message and also log it to the console at the same time.




