# Use the official Ubuntu 16.04 as a base image
FROM ubuntu:16.04

# Set environment variables for non-interactive installation
ENV DEBIAN_FRONTEND=noninteractive

# Update and install necessary packages
RUN apt-get update && apt-get install -y \
    lsb-release \
    gnupg2 \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Add the ROS repository
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# Add the ROS key
RUN curl -sSL 'http://packages.ros.org/ros.key' | apt-key add -

# Update package list and install ROS Kinetic
RUN apt-get update && apt-get install -y \
    ros-kinetic-desktop-full \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init && rosdep update

# Setup environment
RUN echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
SHELL ["/bin/bash", "-c", "source /opt/ros/kinetic/setup.bash"]

# Install dependencies for building ROS packages
RUN apt-get update && apt-get install -y \
    python-rosinstall \
    python-rosinstall-generator \
    python-wstool \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

# Install GCC 7 and set it as the default compiler
RUN add-apt-repository ppa:ubuntu-toolchain-r/test -y && \
    apt-get update && \
    apt-get install -y gcc-7 g++-7 && \
    update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-5 60 --slave /usr/bin/g++ g++ /usr/bin/g++-5 && \
    update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-7 70 --slave /usr/bin/g++ g++ /usr/bin/g++-7 && \
    update-alternatives --config gcc

# Verify GCC version
RUN gcc --version

# Install Eigen3, PCL, and OpenCV
RUN apt-get update && apt-get install -y \
    libeigen3-dev \
    libpcl-dev \
    libopencv-dev \
    && rm -rf /var/lib/apt/lists/*

# Clone and build FIESTA
RUN mkdir -p ~/catkin_ws/src && \
    cd ~/catkin_ws/src && \
    git clone https://github.com/hlx1996/Fiesta.git && \
    cd ~/catkin_ws && \
    /bin/bash -c "source /opt/ros/kinetic/setup.bash && catkin_make"

# Source the setup.bash
RUN echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc