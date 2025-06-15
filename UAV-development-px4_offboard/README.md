# UAV-development
PX4 offboard to control simulation drone with optical flow
# How-to Guide
## PX4 Autopilot
First we must clone the version of PX4 Autopilot (version 1.14)

```shell
git clone https://github.com/PX4/PX4-Autopilot.git --recursive -b release/1.14
```

**Note**: we use the gazebo classic 11 for simulate model uav. Please install `gazebo classic 11` before use this model.
Run this script in a bash shell to install everything (with `--no-sim-tools`)

```shell
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh --no-sim-tools
cd PX4-Autopilot/
make px4_sitl
```
**Modify some file:**
In our project, we have modified and added some file in simulate model to help the model suitable for our project.

Add (replace if had) down_camera, iris, iris_opt_flow and add sonar_ros in `~/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models` with folder in `Additional_file` folder

Replace `dds_topics.yaml` in `~/PX4-Autopilot/src/modules/uxrce_dds_client/dds_topics.yaml` with file in `Additional_file` folder
**(Optional)** You can modify command in `1010_gazebo-classic_iris_opt_flow file` in `~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes` with file in `Additional_file` folder

**Note**: To run model, update plugin is needed for simulation enviroment.

## PX4-ROS2
Clone our packages to use px4_offboard control and optical_flow node
```shell
git clone https://github.com/Lephap129/UAV-development.git -b px4_offboard
cd UAV-development/
colcon build
```
**Note**: You can modify the folder name as your workspace folder after clone it if you want!
## Run simulation
We have just update our project to run all of our project, so just run in new terminal:
```shell
cd UAV-development/
source install/setup.bash #if need
ros2 launch px4_offboard offboard_velocity_control.launch.py
```
## Monitor and log for real flight
We can run monitor for real flight with this command:
```shell
cd UAV-development/
source install/setup.bash #if need
ros2 launch px4_offboard moniter_optical_flow_offboard.launch.py
```

## Development
We want to develop system to run optical flow without GPS location.
We now update the scale of flow in optical flow that suitable for drone updating.
The drone now can hold position, simpy movement, even though it's not so good.
We add a special file `visualizer_real_fly.py` for monitor in real flight.


