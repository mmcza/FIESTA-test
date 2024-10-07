# Use the official Ubuntu 16.04 image
FROM ubuntu:16.04

# Set non-interactive frontend for apt-get
ENV DEBIAN_FRONTEND=noninteractive

# Update and install basic packages
RUN apt-get update && \
    apt-get upgrade -y && \
    apt-get install -y \
    software-properties-common \
    wget \
    git \
    build-essential \
    cmake \
    libeigen3-dev \
    libpcl-dev \
    libopencv-dev

# Add repository for GCC 7
RUN add-apt-repository ppa:ubuntu-toolchain-r/test -y && \
    apt-get update && \
    apt-get install -y gcc-7 g++-7

# Update alternatives to use GCC 7 and G++ 7 as the default
RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-5 60 --slave /usr/bin/g++ g++ /usr/bin/g++-5 && \
    update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-7 70 --slave /usr/bin/g++ g++ /usr/bin/g++-7 && \
    update-alternatives --config gcc

# Check GCC version
RUN gcc --version

# Install ROS Kinetic
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116 && \
    apt-get update && \
    apt-get install -y --allow-unauthenticated ros-kinetic-desktop-full && \
    rosdep init && \
    rosdep update

# Setup ROS environment
RUN echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
SHELL ["/bin/bash", "-c"]
RUN source ~/.bashrc

# Install necessary OpenGL libraries for RViz
RUN apt-get update && \
    apt-get install -y libgl1-mesa-glx libgl1-mesa-dri mesa-utils x11-apps

# Create catkin workspace
RUN mkdir -p ~/catkin_ws/src
WORKDIR /root/catkin_ws/src

# Clone FIESTA repository
RUN git clone https://github.com/hlx1996/Fiesta.git

# Build the workspace
WORKDIR /root/catkin_ws
RUN /bin/bash -c "source /opt/ros/kinetic/setup.bash && catkin_make"

# Source the setup file automatically when starting a new shell
RUN echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

# Set the working directory to the source folder for later use
WORKDIR /root/catkin_ws/src