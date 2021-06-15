# Simulating the Uvify IFO-S Locally with Gazebo/ROS

Currently, this can only be done with Ubuntu 18.04. We need NVIDIA to release an image based on Ubuntu 20.04 for the Jetson Nano, in order to migrate everything to Ubuntu 20.04 and ROS2.

The repos `ifo_hardware` and `ifo_gazebo` (i.e. this repo) are analogies of each other, which take care of setting up and providing the interface to the real and simulated IFO quadcopter, respectively. As such, `ifo_hardware` should only ever be cloned on the IFO's Jetson Nano, whilst `ifo_gazebo` should be cloned on your local computer. 

This repo is a collection of ROS packages, Gazebo models, and plugins. It does not contain the PX4 source code, Gazebo simulator itself, or ROS. These are all dependencies that must be installed as per the instructions below.

## Folder structure

- `./docs/` : A directory for any files related to documentation.
- `./ifo_gazebo/` : The main folder, also a ROS package, which contains all the specifications of the gazebo model, as well as some basic launch files.
- `./realsense_gazebo_plugin/` : A submodule of a realsense plugin made by PAL Robotics. 

## TODO
1. Add IMU data to the realsense model
2. Get an actual CAD of the Uvify IFO-S
3. Try moving PX4 source code to a submodule of this repo
4. Blacklist unnecessary `mavros` plugins
5. Remove need to run `setup_ifo_gazebo.bash` every time.
6. Create a "lite" version of the IFO model which doesnt have cameras and uses optical flow mockup. For speed.
7. Play with PX4 parameters to get the desired PX4 data publishing at the desired rate.
8. Try deleting `mavros`, `mavlink`, and just using a standard installation since we wont be modifying it. 

## Installation - PX4, Gazebo, ROS
The PX4 documentation cites that their installation scripts, which we will be using, are intended for a _clean_ Ubuntu 18.04 installation. It might still work for a non-fresh installation though. 
### Installing PX4
Clone the PX4 source code

```
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
```

The `--recursive` option is very important as it will clone an dinitialize the submodules that PX4 needs. Run the bash script `ubuntu.sh` with no arguments to install everything needed for PX4 development on your local machine.

```
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
```
	
Reboot your computer when complete. Technically, at this point, your computer is also outfitted with a development environment to modify the PX4 source code itself, and do some basic software-in-the-loop (SITL) testing in simulation (not necessarily using Gazebo as the simulator).

See PX4's [Ubuntu Development Environment instructions](https://docs.px4.io/master/en/dev_setup/dev_env_linux_ubuntu.html) for reference.

### Installing ROS, MAVROS, Gazebo
The PX4 documentation conveniently provides an install script which will install all the required software including ROS Melodic, Gazebo 9, MAVROS and more

```
wget https://raw.githubusercontent.com/PX4/Devguide/master/build_scripts/ubuntu_sim_ros_melodic.sh
bash ubuntu_sim_ros_melodic.sh
```

Note that this script should have created the directory `~/catkin_ws/` if it doesnt exist. 

See PX4's [ROS/Gazebo installation instructions](https://docs.px4.io/master/en/dev_setup/dev_env_linux_ubuntu.html#rosgazebo) for reference.

### Testing - Building PX4 and Running Gazebo
Go to the directory where you cloned the PX4 source code

```
cd ~/PX4-Autopilot
```

Then, build the PX4 source code

```
make px4_sitl gazebo
```
	
This can take a while the first time, and should eventually launch PX4 and Gazebo at the end. __Note: this process can require a lot of memory, and can fail if other applications take up too much memory. Close all other applications before running the line above. Try multiple times if necessary.__

Arriving to this point indicates that you have successfully built PX4 and attached it to a stock Gazebo simulation. The next steps describe how to launch our custom IFO-S gazebo model.

## Setting up the custom IFO-S Model
Clone the `ifo_gazebo` repo under `~/catkin_ws/src/`

```
cd ~/catkin_ws/src/
git clone https://YOUR_USERNAME@bitbucket.org/decargroup/ifo_gazebo.git
```
>Note: the above URL is for an HTTPS login to the remote git repo, using SSH-keys when using git is very convenient and recommended.)

Next, compile the C++ Gazebo plugins using `catkin build`.

```
cd ~/catkin_ws/
catkin build
```

After some time, the build process should display that all the packages have succeeded.   

> Note: This may also build the `mavros` source that was added to the `~/catkin_ws/src/` directory by the `ubuntu_sim_ros_melodic.sh` script. This can again take a while and fail a few times due to running out of memory. You should just retry with all other apps closed if this is the case. Currently, we have no intention of modifying the `mavros` source, and we can try deleting the folder under  `~/catkin_ws/src/` and simply installing `mavros` as a regular ros package with
> 
>     sudo apt-get install ros-melodic-mavros ros-melodic-mavros-extras
>
>     wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
>
>     ./install_geographiclib_datasets.sh
> 
> See the official [MAVROS installation instructions](https://github.com/mavlink/mavros/tree/master/mavros#installation). This is a TODO.

Once done, as usual, (and everytime you re-run `catkin build`)

```
source ~/catkin_ws/devel/setup.bash 
```

Alternatively, if you run

```
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

then this source command will execute every time a new terminal is open.


>**Note:  From now on, and forever, you should use `catkin build` instead of `catkin_make` to build a ROS workspace. Its newer, prettier, and more versatile. Check out the [documentation for catkin](https://catkin-tools.readthedocs.io/en/latest/verbs/catkin_build.html).**



### Testing - Simulating the IFO-S in Gazebo, running PX4 and ROS.
Now, with all the bare minimum code compiled, we should be ready to run simulation. There's actually one more setup script we need to run which simply adds a bunch of gazebo plugins located in the `~/PX4-Autopilot/` directory to the Gazebo plugin search path. 

```
source ~/catkin_ws/src/ifo_gazebo/ifo_gazebo/setup_ifo_gazebo.bash ~/PX4-Autopilot
```

> Note: the above command needs to be run for every new terminal that needs to run the following launch file. 
>
> To get rid of this step, we can consider fixing the location of the `~/PX4-Autopilot/` directory and adding this to the `PX4_CLONE_DIR` as a permanent environment variable, and then also copying the commands located inside the above `setup.bash` script into `~/.bashrc`. I'm still wondering if there is a more "portable". 
>
> The ideal solution, in my opinion, is to have the PX4 source be a submodule of this repo. That way, the PX4 clone directory is fixed relative to this gazebo simulation. Also, the PX4 source is very much a dependency, so it makes sense. We would have everything needed to create a simulated version of our quadrotor in one folder. The only issues is that PX4 cannot be compiled with `catkin make` so it will have to be "blacklisted" from `catkin build` ([that functionality is available](https://catkin-tools.readthedocs.io/en/latest/verbs/catkin_config.html#whitelisting-and-blacklisting-packages)) and built manually as needed. TODO.

You can now launch the simulation with

```
roslaunch ifo_gazebo ifo_empty_world.launch
```

The Gazebo GUI should start up, and PX4 should launch. 

![Gazebo model of IFO-S using Iris Quadcopter as baseline.](./docs/ifo_visual.png)

If you open another terminal and run `rostopic list` you should see a bunch of topics.

```
$ rostopic list
/clock
/diagnostics
/gazebo/link_states
/gazebo/model_states
/gazebo/parameter_descriptions
/gazebo/parameter_updates
/gazebo/set_link_state
/gazebo/set_model_state
/mavlink/from
/mavlink/gcs_ip
/mavlink/to
/mavros/actuator_control
/mavros/adsb/send
/mavros/adsb/vehicle
/mavros/altitude
/mavros/battery
/mavros/cam_imu_sync/cam_imu_stamp
/mavros/companion_process/status
/mavros/debug_value/debug
/mavros/debug_value/debug_vector
/mavros/debug_value/named_value_float
/mavros/debug_value/named_value_int
/mavros/debug_value/send
/mavros/esc_info
/mavros/esc_status
/mavros/estimator_status
/mavros/extended_state
/mavros/fake_gps/mocap/tf
/mavros/global_position/compass_hdg
/mavros/global_position/global
/mavros/global_position/gp_lp_offset
/mavros/global_position/gp_origin
/mavros/global_position/home
/mavros/global_position/local
/mavros/global_position/raw/fix
/mavros/global_position/raw/gps_vel
/mavros/global_position/raw/satellites
/mavros/global_position/rel_alt
/mavros/global_position/set_gp_origin
/mavros/gps_rtk/rtk_baseline
/mavros/gps_rtk/send_rtcm
/mavros/gpsstatus/gps1/raw
/mavros/gpsstatus/gps1/rtk
/mavros/gpsstatus/gps2/raw
/mavros/gpsstatus/gps2/rtk
/mavros/hil/actuator_controls
/mavros/hil/controls
/mavros/hil/gps
/mavros/hil/imu_ned
/mavros/hil/optical_flow
/mavros/hil/rc_inputs
/mavros/hil/state
/mavros/home_position/home
/mavros/home_position/set
/mavros/imu/data
/mavros/imu/data_raw
/mavros/imu/diff_pressure
/mavros/imu/mag
/mavros/imu/static_pressure
/mavros/imu/temperature_baro
/mavros/imu/temperature_imu
/mavros/landing_target/lt_marker
/mavros/landing_target/pose
/mavros/landing_target/pose_in
/mavros/local_position/accel
/mavros/local_position/odom
/mavros/local_position/pose
/mavros/local_position/pose_cov
/mavros/local_position/velocity_body
/mavros/local_position/velocity_body_cov
/mavros/local_position/velocity_local
/mavros/log_transfer/raw/log_data
/mavros/log_transfer/raw/log_entry
/mavros/manual_control/control
/mavros/manual_control/send
/mavros/mission/reached
/mavros/mission/waypoints
/mavros/mocap/pose
/mavros/mount_control/command
/mavros/mount_control/orientation
/mavros/obstacle/send
/mavros/odometry/in
/mavros/odometry/out
/mavros/onboard_computer/status
/mavros/param/param_value
/mavros/play_tune
/mavros/px4flow/ground_distance
/mavros/px4flow/raw/optical_flow_rad
/mavros/px4flow/raw/send
/mavros/px4flow/temperature
/mavros/radio_status
/mavros/rc/in
/mavros/rc/out
/mavros/rc/override
/mavros/setpoint_accel/accel
/mavros/setpoint_attitude/cmd_vel
/mavros/setpoint_attitude/thrust
/mavros/setpoint_position/global
/mavros/setpoint_position/global_to_local
/mavros/setpoint_position/local
/mavros/setpoint_raw/attitude
/mavros/setpoint_raw/global
/mavros/setpoint_raw/local
/mavros/setpoint_raw/target_attitude
/mavros/setpoint_raw/target_global
/mavros/setpoint_raw/target_local
/mavros/setpoint_trajectory/desired
/mavros/setpoint_trajectory/local
/mavros/setpoint_velocity/cmd_vel
/mavros/setpoint_velocity/cmd_vel_unstamped
/mavros/state
/mavros/statustext/recv
/mavros/statustext/send
/mavros/target_actuator_control
/mavros/time_reference
/mavros/timesync_status
/mavros/trajectory/desired
/mavros/trajectory/generated
/mavros/trajectory/path
/mavros/vfr_hud
/mavros/vision_pose/pose
/mavros/vision_pose/pose_cov
/mavros/vision_speed/speed_twist_cov
/mavros/wind_estimation
/realsense/color/camera_info
/realsense/color/image_raw
/realsense/color/image_raw/compressed
/realsense/color/image_raw/compressed/parameter_descriptions
/realsense/color/image_raw/compressed/parameter_updates
/realsense/color/image_raw/compressedDepth
/realsense/color/image_raw/compressedDepth/parameter_descriptions
/realsense/color/image_raw/compressedDepth/parameter_updates
/realsense/color/image_raw/theora
/realsense/color/image_raw/theora/parameter_descriptions
/realsense/color/image_raw/theora/parameter_updates
/realsense/depth/camera_info
/realsense/depth/color/points
/realsense/depth/image_raw
/realsense/depth/image_raw/compressed
/realsense/depth/image_raw/compressed/parameter_descriptions
/realsense/depth/image_raw/compressed/parameter_updates
/realsense/depth/image_raw/compressedDepth
/realsense/depth/image_raw/compressedDepth/parameter_descriptions
/realsense/depth/image_raw/compressedDepth/parameter_updates
/realsense/depth/image_raw/theora
/realsense/depth/image_raw/theora/parameter_descriptions
/realsense/depth/image_raw/theora/parameter_updates
/realsense/infra1/camera_info
/realsense/infra1/image_raw
/realsense/infra1/image_raw/compressed
/realsense/infra1/image_raw/compressed/parameter_descriptions
/realsense/infra1/image_raw/compressed/parameter_updates
/realsense/infra1/image_raw/compressedDepth
/realsense/infra1/image_raw/compressedDepth/parameter_descriptions
/realsense/infra1/image_raw/compressedDepth/parameter_updates
/realsense/infra1/image_raw/theora
/realsense/infra1/image_raw/theora/parameter_descriptions
/realsense/infra1/image_raw/theora/parameter_updates
/realsense/infra2/camera_info
/realsense/infra2/image_raw
/realsense/infra2/image_raw/compressed
/realsense/infra2/image_raw/compressed/parameter_descriptions
/realsense/infra2/image_raw/compressed/parameter_updates
/realsense/infra2/image_raw/compressedDepth
/realsense/infra2/image_raw/compressedDepth/parameter_descriptions
/realsense/infra2/image_raw/compressedDepth/parameter_updates
/realsense/infra2/image_raw/theora
/realsense/infra2/image_raw/theora/parameter_descriptions
/realsense/infra2/image_raw/theora/parameter_updates
/rosout
/rosout_agg
/tf
/tf_static
```

# Version 2: Fully portable
In this version we moved the `PX4-Autopilot` repo to be a submodule of this repo. This allows the PX4 directory to be fixed relative to the gazebo model, which simplifies the simulator start-up procedure since the user no longer needs to specify the location of PX4 source code. The `PX4-Autopilot` repo has lots of submodules itself, so it is critical to clone this repo with the `--recursive` flag. Start by creating the folder `~/catkin_ws/src/`. Then

    cd ~/catkin_ws/src
    git clone https://bitbucket.org/decargroup/ifo_gazebo.git --recursive

Feel free to change the above URL to one appropriate for SSH-keys. Alternatively, you can clone this repo regularly and then run `git submodule update --init --recursive`.

Next, we must blacklist the `PX4-Autopilot` directory in the catkin tools, so that the `catkin build` command does not compile the PX4 source code (since it fails). PX4 must be built manually. While inside your catkin workspac, add the `px4` package to the blacklist,

    catkin config --blacklist px4

You can now run 

    catkin build

and test to see that all the packages get built successfully, with the px4 package being skipped. To compile PX4 manually, we must first install the toolchain. Thankfully PX4 provides an install script which installs everything required

    bash ~/catkin_ws/src/PX4-Autopilot/Tools/setup/ubuntu.sh

Then, build the PX4 code. 

    cd ~/catkin_ws/src/ifo_gazebo/PX4-Autopilot
    make px4_sitl gazebo

This can take a long time, and can fail multiple times. The reason of failure is apparently due to your computer running out of RAM. Nevertheless, the build process gets a little further every time, so the solution is just to keep running `make px4_sitl gazebo` until it succeeds. Close all other programs to make it go faster. Once successful, you should see the PX4 app start up in the terminal and the Gazebo GUI launching. You should only need to do this build step once, provided that you never modify the PX4 source code. Next, the PX4 documentation conveniently provides an install script which will install all the required software for ROS/Gazebo simulation including ROS Melodic, Gazebo 9, MAVROS and more

    wget https://raw.githubusercontent.com/PX4/Devguide/master/build_scripts/ubuntu_sim_ros_melodic.sh
    bash ubuntu_sim_ros_melodic.sh

Note that this will create two additional folders: `~/catkin_ws/src/mavros/` and `~/catkin_ws/src/mavros_extras/`. __Delete these folders completely,__ since at this point we have no intention of modifying the `mavros` and `mavros_extras` ROS packages, and there is no need to recompile them every time we run `catkin build`. Instead, we will just install these packages like regular ROS packages with 

    sudo apt-get install ros-melodic-mavros ros-melodic-mavros-extras

As per the official [MAVROS installation instructions](https://github.com/mavlink/mavros/tree/master/mavros#installation), we also need to install some geographic dataset dependency for proper reference frame conversions.

``` bash
cd ~
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
./install_geographiclib_datasets.sh
rm install_geographiclib_datasets.sh     # Delete the file once done.
```

Finally, we need to source the usual setup script in `catkin_ws/devel/setup.bash`, but also a custom script `setup_ifo_gazebo.bash` located in this repo, which manually specifies the paths to relevant dependencies inside the PX4 source using environment variables. These two scripts need to be run for every new terminal. Alternatively, adding them to `~/.bashrc` will automatically execute them with every new terminal

``` bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
echo "source ~/catkin_ws/src/ifo_gazebo/setup_ifo_gazebo.bash suppress" >> ~/.bashrc
```

Then, you should be ready to fire up the simulator

    roslaunch ifo_gazebo ifo_empty_world.launch

The PX4 app should be running in the terminal, and the Gazebo GUI should have started, displaying a single quadcopter located at the origin.