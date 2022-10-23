# Quick start

## Installation and configuration

### Docker

[Moveit and Docker: best practices](https://picknik.ai/ros/robotics/docker/2021/07/20/Vatan-Aksoy-Tezer-Docker.html)

#### Installation

Follow instructions from the Docker [docs](https://docs.docker.com/engine/install/ubuntu/)

Follow the first chapter of [linux post installation instructions](https://docs.docker.com/engine/install/linux-postinstall/), "manage docker as non-root user"

To enable the Nvidia driver inside containers follow the Nvidia [docs](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html) (Nvidia driver is not compatible with preempt_rt kernel)

&nbsp;

Test your installation by running

```shell
docker run hello-world
```

with Nvidia

```shell
docker run --rm --gpus all nvidia/cuda:11.0.3-base-ubuntu20.04 nvidia-smi
```

&nbsp;

#### Docker commands

Familiarize yourself with the following Docker CLI commands

- [docker build](https://docs.docker.com/engine/reference/commandline/build/)
- [docker run](https://docs.docker.com/engine/reference/commandline/run/)
- [docker image ls](https://docs.docker.com/engine/reference/commandline/image_ls/)
- [docker image prune](https://docs.docker.com/engine/reference/commandline/image_prune/)
- [docker container list](https://docs.docker.com/engine/reference/commandline/container_ls/)
- [docker container prune](https://docs.docker.com/engine/reference/commandline/container_prune/)

&nbsp;

#### Vscode

Install [vs code](https://code.visualstudio.com/) and the following vs code extensions

- [docker](https://code.visualstudio.com/docs/containers/overview)
- [remote development](https://code.visualstudio.com/docs/remote/remote-overview)

devcontainer.json provided in .docker folder

&nbsp;

#### Set up required permissions on host for interfacing with hardware

- Process priority for UR driver

    When not using a realtime kernel it is recommended to run the UR driver with high process priority.

    See process [nice](https://en.wikipedia.org/wiki/Nice_(Unix)) for more information on process niceness.

    This requires the user to have the permissions to change the niceness of processes.

    Add the following to /etc/security/limits.conf on the Docker host

    `$USER - nice -15`

    Once inside the container run `su $USER` in order for the permissions to be loaded.

- Udev rules for Realsense Driver

    Download the [udev rules](https://github.com/IntelRealSense/librealsense/blob/master/config/99-realsense-libusb.rules) and place them in /etc/udev/rules.d  

    Run ```sudo service udev reload``` and ```sudo service udev restart``` to load the new rules, not required if you reboot

Recommended to reboot at this point to ensure all changes are applied

&nbsp;

## Use

&nbsp;

### Worflow

clone repo -> build image -> run image

### fork and clone repo

&nbsp;

### Building the image

Running the following command from the root of the repo will execute the build image shell script

```shell
.docker/build_image.sh
```

&nbsp;

### Running the image

note: On Ubuntu 20.04 --privileged flag is required on Ubuntu 22 it can be omitted

Running the following command from the root of the repo will execute the run image shell script

```shell
.docker/run_user.sh
```

With Nvidia driver

```shell
.docker/run_user_nvidia.sh
```

Once inside the container, ake ownership of the workspace with

```shell
sudo chown -R $USER /dev_ws
```

Open vscode, go to the docker tab.
Select the running container, right click and select attach vscode

&nbsp;

### Using the container as a development environment

#### Terminal

Terminator is installed in the container for multiple terminals launch terminator from the CLI inside the container. Run `terminator` to start. You will need to source your ros environment.

For automatic sourcing of ros environment add the following to your .bashrc or .zshrc

bash

```bash
if [[ "$PWD" = "/dev_ws" && -r /dev_ws/setup.bash ]] || [[ "$PWD" = "/dev_ws/src" && -r /dev_ws/setup.bash ]]; then
  source /dev_ws/setup.bash
fi
```

zsh

```zsh
if [[ "$PWD = /dev_ws" && -r /dev_ws/setup.zsh ]]||[[ "$PWD = /dev_ws/src" && -r /dev_ws/setup.zsh ]]; then
  source /dev_ws/setup.zsh
fi
```

This will source the workspace for every new shell opened in  /dev_ws or /dev_ws/src

&nbsp;

#### Attach vs code to container

In vs code go to the Docker tab in the side bar. Right click on the container named moveit1_ur:latest. Select attach vscode.  

When attaching to the container for first time:  
In vs code open the command palette (Ctrl-Shift-P). Select Remote-containers: Open attached container configuration file. Copy paste content of devcontainer.json and save. Close the vscode window and reattach.

&nbsp;

#### Saving

Before closing the terminal you ran the docker image from remember to commit and push your changes.

### Connecting to the robot

The bringup launch file will attempt to start the UR driver with hight process priority.  

Ensure both the user on the host and container have the required permissions to do so.  
See section "Process priority for UR driver"

These instructions assume

- Robot IP: 192.168.56.101
- Docker Host IP: 192.168.56.1

Launch the robot bringup, this file sets the robot IP and loads the kinematics calibration for the IAAC UR10e.  

- without endeffector

    ```shell
    ur10e_moveit_config ur10e_iaac_bringup.launch 
    ```

- with endeffector

    ```shell
    ur10e_ee_moveit_config ur10e_ee_iaac_bringup.launch 
    ```

On the ur pendant open the program named ros and press play

If URcaps fails to connect add the following rule to the firewall (ufw) on the docker host u

```shell
sudo ufw allow from 192.168.56.101 to 192.168.56.1
```

When the robot is connected you should see the following in the terminal you launched the bringup launchfile from.

```shell
[ INFO]  Sent program to robot
[ INFO]  Robot connected to reverse interface. Ready to receive control commands.
```

You can use the `top` command to check the ur driver is running unnicely

&nbsp;

## Without docker

Install ROS [noetic](http://wiki.ros.org/noetic/Installation)

Install Python linter, formatter and ROS pkg dependencies

```shell
pip3 install -U \
    argcomplete \
    black \
    flake8-blind-except \
    flake8-builtins \
    flake8-class-newline \
    flake8-comprehensions \
    flake8-deprecated \
    flake8-return \
    flake8-length \
    flake8-todo \
    flake8-quotes \
    numpy \
    open3d \
    pyquaternion
```

Ensure the following ros pkg's and dependencies are installed.

```shell
apt update && apt install -y \
    python3-pip \
    python3-flake8 \
    build-essential \
    cmake \
    pkg-config \
    python3-catkin-tools \
    python3-rosdep \
    python3-rosinstall-generator \
    python3-vcstool \
    ros-noetic-rqt \
    ros-noetic-rqt-action \
    ros-noetic-rqt-console \
    ros-noetic-rqt-service-caller \
    ros-noetic-rqt-graph \
    ros-noetic-rqt-topic \
    ros-noetic-rqt-top \
    ros-noetic-rqt-joint-trajectory-controller \
    ros-noetic-moveit \
    ros-noetic-trac-ik-kinematics-plugin \
    ros-noetic-ur-client-library \
    ros-noetic-ur-msgs \
    ros-noetic-controller-manager \
    ros-noetic-pass-through-controllers \
    ros-noetic-force-torque-sensor-controller \
    ros-noetic-industrial-robot-status-interface \
    ros-noetic-industrial-robot-status-controller \
    ros-noetic-joint-state-controller \
    ros-noetic-joint-trajectory-controller \
    ros-noetic-cartesian-trajectory-controller \
    ros-noetic-scaled-joint-trajectory-controller \
    ros-noetic-speed-scaling-interface \
    ros-noetic-speed-scaling-state-controller \
    ros-noetic-velocity-controllers \
    ros-noetic-effort-controllers \
    ros-noetic-kdl-parser \
    ros-noetic-roslint \
    ros-noetic-rqt-gui \
    ros-noetic-rqt-gui-py \
    ros-noetic-rqt-py-common \
    python3-matplotlib \
    ros-noetic-ros-industrial-cmake-boilerplate \
    ros-noetic-rqt-image-view \
    software-properties-common \
    ros-noetic-ddynamic-reconfigure \
    ros-noetic-rgbd-launch \
```

For STOMP motion planner, you will need to install nlopt. Follow installation instructions on [nlopt github page](https://github.com/stevengj/nlopt).

For Realsense D405

```
apt-key adv \
    --keyserver keyserver.ubuntu.com \
    --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || \
    apt-key adv \
    --keyserver hkp://keyserver.ubuntu.com:80 \
    --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE \
    && add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u \
    && apt-get install -y --install-recommends \
    librealsense2-dkms librealsense2-utils \
    librealsense2-dev librealsense2-dbg
```

Follow "Set up required permissions on host for interfacing with hardware" in docker section"

Create a catkin workspace and clone the following pkgs to the src directory.

- [ros industrial stomp](https://github.com/ros-industrial/stomp)
- [ros industrial stomp_ros](https://github.com/ros-industrial/stomp_ros)
- [rqt joint trajectory](https://github.com/tork-a/rqt_joint_trajectory_plot)
- [realsense-ros](https://github.com/rjwb1/realsense-ros.git)

Fork and clone this repo and clone to src directory.

Run `catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release`  
Run `catkin build`  
