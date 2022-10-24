ARG ROSDISTRO=noetic

FROM ros:$ROSDISTRO-robot

ARG ROSDISTRO

# Set environment variables
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV DEBIAN_FRONTEND=noninteractive

ENV TERM xterm-256color

# Install tools
RUN apt-get update && apt-get install -y \
    ssh \
    git \
    curl \
    terminator \
    python3-pip \
    python3-flake8 \
    build-essential \
    cmake \
    iputils-ping 

RUN pip3 install -U --no-cache-dir\
    argcomplete \
    flake8-blind-except \
    flake8-builtins \
    flake8-class-newline \
    flake8-comprehensions \
    flake8-deprecated \
    flake8-return \
    flake8-length \
    flake8-todo \
    flake8-quotes \
    jupyterlab_widgets \
    black \
    numpy \
    open3d \
    pyquaternion

# Intall ROS pkgs and dependencies
RUN apt-get update && apt-get install -y \
    pkg-config \
    python3-catkin-tools \
    python3-rosdep \
    python3-rosinstall-generator \
    python3-vcstool \
    ros-$ROSDISTRO-rqt \
    ros-$ROSDISTRO-rqt-action \
    ros-$ROSDISTRO-rqt-console \
    ros-$ROSDISTRO-rqt-service-caller \
    ros-$ROSDISTRO-rqt-graph \
    ros-$ROSDISTRO-rqt-topic \
    ros-$ROSDISTRO-rqt-top \
    ros-$ROSDISTRO-rqt-reconfigure \
    ros-$ROSDISTRO-rqt-joint-trajectory-controller \
    ros-$ROSDISTRO-moveit \
    ros-$ROSDISTRO-trac-ik-kinematics-plugin \  
    # UR dependencies
    ros-$ROSDISTRO-ur-client-library \
    ros-$ROSDISTRO-ur-msgs \
    ros-$ROSDISTRO-controller-manager \
    ros-$ROSDISTRO-pass-through-controllers \
    ros-$ROSDISTRO-force-torque-sensor-controller \
    ros-$ROSDISTRO-industrial-robot-status-interface \
    ros-$ROSDISTRO-industrial-robot-status-controller \
    ros-$ROSDISTRO-joint-state-controller \
    ros-$ROSDISTRO-joint-trajectory-controller \
    ros-$ROSDISTRO-cartesian-trajectory-controller \
    ros-$ROSDISTRO-scaled-joint-trajectory-controller \
    ros-$ROSDISTRO-speed-scaling-interface \
    ros-$ROSDISTRO-speed-scaling-state-controller \
    ros-$ROSDISTRO-velocity-controllers \
    ros-$ROSDISTRO-effort-controllers \
    ros-$ROSDISTRO-kdl-parser \
    # RQT joint trajectory plot dependencies
    ros-$ROSDISTRO-roslint \
    ros-$ROSDISTRO-rqt-gui \
    ros-$ROSDISTRO-rqt-gui-py \
    ros-$ROSDISTRO-rqt-py-common \
    python3-matplotlib \
    # STOMP dependencies
    ros-$ROSDISTRO-ros-industrial-cmake-boilerplate \
    # Realsense dependencies
    software-properties-common \
    ros-$ROS_DISTRO-rqt-image-view \
    ros-$ROS_DISTRO-ddynamic-reconfigure \
    ros-$ROS_DISTRO-rgbd-launch 
    
# Add repo and install realsense pkgs
RUN apt-key adv \
    --keyserver keyserver.ubuntu.com \
    --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || \
    apt-key adv \
    --keyserver hkp://keyserver.ubuntu.com:80 \
    --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE \
    && add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u \
    && apt-get install -y --install-recommends \
    librealsense2-dkms librealsense2-utils \
    librealsense2-dev librealsense2-dbg

# Reduce size by removing apt cache
RUN ["/bin/bash", "-c", "rm -rf /var/lib/apt/lists/*"]

# Add nice permision to container
RUN echo "* - nice -15" >> /etc/security/limits.conf

# build STOMP dependency nlopt
RUN ["/bin/bash", "-c", "git clone https://github.com/stevengj/nlopt.git &&\
    cd nlopt &&\
    mkdir build &&\
    cd build &&\
    cmake .. &&\
    make &&\
    make install"]

# Create workspace
RUN mkdir -p dev_ws/src
WORKDIR /dev_ws/src

# Clone STOMP and rqt_joint_trajectory pkgs
RUN ["/bin/bash", "-c", "git clone https://github.com/ros-industrial/stomp.git &&\
    git clone https://github.com/ros-industrial/stomp_ros.git &&\
    git clone https://github.com/tork-a/rqt_joint_trajectory_plot.git &&\
    git clone https://github.com/rjwb1/realsense-ros.git"]

# Copy local pkg's from host to container
COPY . /dev_ws/src/

# Build catkin workspace
WORKDIR /dev_ws

RUN ["/bin/bash", "-c", "source /opt/ros/$ROSDISTRO/setup.bash &&\
    catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release &&\
    catkin build \
    "]

# Copy and make setup script executable
COPY .docker/setup.bash /dev_ws/setup.bash
RUN ["/bin/bash", "-c", " chmod 777 /dev_ws/setup.bash"]

# Copy and make entrypoint script executable
RUN ["/bin/bash", "-c", "cp /dev_ws/src/.docker/entrypoint.sh / && chmod 777 /entrypoint.sh"]
ENTRYPOINT [ "/entrypoint.sh" ]

# Start bash shell
CMD ["bash"]