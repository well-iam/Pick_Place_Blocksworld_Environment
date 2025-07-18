# Pick and Place in a Blocksworld Environment using HTN Planning and MoveIt

## :package: About

This package contains the developed code for the final project of the Planning and Navigation 2024/25 Course. The authors of the package are:
Chiara Panagrosso, Roberto Rocco, William Notaro. Here's an example video

https://private-user-images.githubusercontent.com/182740140/467962562-0d6c1e1e-ce1d-46a5-9862-abccb1980fe2.mp4?jwt=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJnaXRodWIuY29tIiwiYXVkIjoicmF3LmdpdGh1YnVzZXJjb250ZW50LmNvbSIsImtleSI6ImtleTUiLCJleHAiOjE3NTI4MzIxMDMsIm5iZiI6MTc1MjgzMTgwMywicGF0aCI6Ii8xODI3NDAxNDAvNDY3OTYyNTYyLTBkNmMxZTFlLWNlMWQtNDZhNS05ODYyLWFiY2NiMTk4MGZlMi5tcDQ_WC1BbXotQWxnb3JpdGhtPUFXUzQtSE1BQy1TSEEyNTYmWC1BbXotQ3JlZGVudGlhbD1BS0lBVkNPRFlMU0E1M1BRSzRaQSUyRjIwMjUwNzE4JTJGdXMtZWFzdC0xJTJGczMlMkZhd3M0X3JlcXVlc3QmWC1BbXotRGF0ZT0yMDI1MDcxOFQwOTQzMjNaJlgtQW16LUV4cGlyZXM9MzAwJlgtQW16LVNpZ25hdHVyZT04NDE2NmU0MWQxNDBkNTM3ZjdkMmMzNmU1ZGZiNjlkODA4YTJmZmFlZDQzYjFkZGM1ZDg0N2ExMWVjYmRhNjBiJlgtQW16LVNpZ25lZEhlYWRlcnM9aG9zdCJ9.PxCx_JmYlHH0dlaoo6u91ZOYYkAP6NSsMMZrTjmxWW8

## :clipboard: Requirements
This package **requires MoveIt2 to be pre-installed** on your system. 
:point_right: Follow the [official MoveIt2 installation guide](https://moveit.picknik.ai/humble/doc/tutorials/getting_started/getting_started.html) to set it up properly.
> :bangbang: This package will **not work** without a working MoveIt installation.

# :hammer: Build
1. Clone the repository in the `src` folder of your ROS 2 workspace.  If you want to only clone the content files without creating the repo folder, use
```
git clone https://github.com/well-iam/Pick_Place_Blocksworld_Environment.git .
```
2. Build the workspace
```
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```
3. Source the setup files
```
source ~/ros2_ws/install/setup.bash
```

# :whale: Alternative Setup with Docker (No Local MoveIt2 Installation Required)
If you'd prefer not to install MoveIt2 manually, you can run this project inside a Docker container with all necessary dependencies pre-configured.

## :gear: Prerequisites
Docker installed on your system. You can get it from the [official Docker website](https://docs.docker.com/get-docker/). 

### :bulb: Notes: 
Superuser permissions are requested. If you want to avoid invoking sudo you can add the currently logged-in user to the docker group


### \:hammer\_and\_wrench: Using the Prebuilt Docker Image

1. Clone the repository containing Dockerfile and scripts to build and tun the image into a local folder:

```
git clone https://github.com/well-iam/MoveIt-Docker.git
cd MoveIt-Docker
```
2. Inside the terminal of the cloned repo make all the bash files executable:
```sh
chmod +x *.sh
```

3. Build the image: 

```
./docker_build_image.sh <IMAGE_NAME>
```
where <IMAGE_NAME> is a name for the image you want to build. The build will take at least 1 hour.
If you have up to 32Gb of RAM you and want to speed building of the image you can substitute inside the Dockerfile the following line:
```
MAKEFLAGS="-j4 -l1" colcon build --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release
```
with:
```
 colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

4. Run the container:

```
./docker_run_container.sh <IMAGE_NAME> <CONTAINER_NAME> <FOLDER_NAME>
```
where <IMAGE_NAME> is the name of the image you have just built, while <CONTAINER_NAME> is a name for the container hosting the image. <FOLDER_NAME> is the name of the local folder you want to mount inside the container ROS2 ws. In order to run the application it must correspond to the name of the folder where **Pick_Place_Blocksworld_Environment** has been cloned.


5. Once inside the container, build and source the workspace:

```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

In the case of stderrs for some packages, keep colcon building until all packages are built.

### :bulb: Notes

To connect to the container from another terminal, run
```
./docker_connect.sh 
```
and select the container you want to connect to.


# :rocket: Launching the Application

## :white_check_mark: Usage
1. Run the RVIZ environment and Gazebo simulation through the launch file:
```
ros2 launch ur_description setup.launch.py 
```
Gazebo simulation starts in the "PAUSED" state. To make it play click on the bottom left "Play" button. You have 5 seconds (a default timeout value difficult to change) to start the Gazebo program, connect the controller and play the simulation.

2. Open a second terminal and run the MoveIt Task Constructor and Move It planner using the following executable:
```
ros2 run mtc_package manager
```
3. Once the console shows the message from `mtc_node` saying **"Waiting for the plan..."**, in another terminal you can launch the **HTN Planner**, which will generate the sequence of tasks to be performed, with the following:

```
ros2 launch blocksword_planner htn_launch.py 
``` 

## :warning: Warning
During the launch of the Gazebo environment you could face a list of errors similar to:
```diff
- [GUI] [Err] [SystemPaths.cc:378] Unable to find file with URI [model://robotiq_description/meshes/visual/2f_85/robotiq_base.dae]

```
This is caused by the unsuccessful update of the GZ_SIM_RESOURCE_PATH environment variable. To fix the error use after sourcing the setup files:
```
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:~/ros2_ws/src/ros2_robotiq_gripper/
```

