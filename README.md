
# Pick and Place in a Blocksworld Environment using HTN Planning and MoveIt

## :package: About

This package contains the developed code for the final project of the Planning and Navigation 2024/25 Course. The authors of the package are:
Chiara Panagrosso, Roberto Rocco, William Notaro

## :clipboard: Requirements
This package **requires MoveIt2 to be pre-installed** on your system. 
Follow the [official MoveIt2 installation guide](https://moveit.picknik.ai/humble/doc/tutorials/getting_started/getting_started.html) to set it up properly.
> :bangbang: This package will **not work** without a working MoveIt installation.

# :hammer: Build
1. Clone the repository in the `src` folder of your ROS 2 workspace.  If you want to only clone the content files without creating the repo folder, use
```
git clone https://github.com/well-iam/Pick_Place_Blocksworld_Environment.git -b without_gripper
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

### \:hammer\_and\_wrench: Using the Prebuilt Docker Image

1. Clone the repository containing Dockerfile and scripts to build and tun the image into a local folder:

```
git clone https://github.com/well-iam/MoveIt-Docker.git
cd MoveIt-Docker
```
2. Inside the terminal of the cloned repo make all the bash files executable:
```sh
$ chmod +x *.sh
```

3. Build the image: 

```
$ ./docker_build_image.sh <IMAGE_NAME>
```
where <IMAGE_NAME> is a name for the image you want to build. The build will take at least 1 hour.
If you have up to 16Gb of RAM you and want to speed building of the image you can substitute inside the Dockerfile the following line:
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

```
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## :white_check_mark: Usage
1. Run the RVIZ environment and Gazebo simulation through the launch file:
```
ros2 launch ur_description setup.launch.py 
```
Gazebo simulation starts in the "PAUSED" state. To make it play click on the bottom left "Play" button. You have 5 seconds (a default timeout value difficult to change) to start the Gazebo program, connect the controller and play the simulation.

2. Run the MoveIt Task Constructor and planner using the following launch file:
```
ros2 launch mtc_package pick_place.launch.py
```
This launch will also loead the bridge for the Gazebo Service  `/SetEntityPose` to ROS nodes.

3. Once the console shows the message from `mtc_node` saying **"Parsing the plan..."**, you can launch the **HTN Planner**, which will generate the sequence of tasks to be performed, with the following:

```
ros2 launch blocksword_planner htn_launch.py 
``` 

