# Docker instructions

This container will access to the users home directory and logged in as the user with their password and x sever access.  
The .docker folder of this repo contains convenience hell scripts for building and running the Docker container. These should be run from the root of the repo.

### Build Image

Running the following command from the root of the repo will execute the build image shell script

```shell
.docker/build_image.sh
```

Or

```shell
docker build --pull --rm -f ./.docker/Dockerfile  -t moveit1_ur:latest
```

### Run Image

note: On Ubuntu 20.04 --privileged flag is required on Ubuntu 22 it can be omitted

Running the following command from the root of the repo will execute the run image shell script

```shell
.docker/run_user.sh
```

or

```shell
docker run -it \
    --user=$(id -u $USER):$(id -g $USER) \
    --group-add sudo \
    --env="DISPLAY" \
    --env=QT_X11_NO_MITSHM=1 \
    --workdir="/dev_ws" \
    --volume="/home/$USER:/home/$USER" \
    --volume="/etc/group:/etc/group:ro" \
    --volume="/etc/passwd:/etc/passwd:ro" \
    --volume="/etc/shadow:/etc/shadow:ro" \
    --volume="/etc/sudoers.d:/etc/sudoers.d:ro" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --net=host \
    --cap-add=sys_nice \
    moveit1_ur:latest
```

For nvidia-docker

```shell
.docker/run_user_nvidia.sh
```

or

```shell
docker run -it \
    --user=$(id -u $USER):$(id -g $USER) \
    --group-add sudo \
    --env="DISPLAY" \
    --env=QT_X11_NO_MITSHM=1 \
    --workdir="/dev_ws" \
    --volume="/home/$USER:/home/$USER" \
    --volume="/etc/group:/etc/group:ro" \
    --volume="/etc/passwd:/etc/passwd:ro" \
    --volume="/etc/shadow:/etc/shadow:ro" \
    --volume="/etc/sudoers.d:/etc/sudoers.d:ro" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --net=host \
    --privileged \
    --cap-add=sys_nice \
    --gpus 'all,"capabilities=compute,display,graphics,utility"' \
    moveit1_ur:latest
```

### Set up required permissions on host for interfacing with hardware

#### Process priority for UR driver

When not using a realtime kernel it is recommended to run the UR driver with high process priority. The bringup launch file will attempt to start the UR driver with hight process priority.

See process [nice](https://en.wikipedia.org/wiki/Nice_(Unix)) for more information on process niceness.

This requires the user to have the permissions to change the niceness of processes.

Add the following to /etc/security/limits.conf on the Docker host

`$USER - nice -15`

Once inside the container run `su $USER` in order for the permissions to be loaded.

#### Udev rules for Realsense Driver

Download the [udev rules](https://github.com/IntelRealSense/librealsense/blob/master/config/99-realsense-libusb.rules) and place them in /etc/udev/rules.d  

### Connecting to the robot

The bringup launch file will attempt to start the UR driver with hight process priority.  

Ensure both the user on the host and container have the required permissions to do so.  
See section "Process priority for UR driver"

The launch files assume

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

On the ur pendant start URcaps

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
