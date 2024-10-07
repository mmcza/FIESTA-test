# Use the image with ROS kinetic installed
FROM osrf/ros:noetic-desktop-full

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

# Install GCC 7 and G++ 7
RUN apt-get update && \
    apt-get install -y gcc-7 g++-7

# Update alternatives to use GCC 7 and G++ 7 as the default
RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-9 60 --slave /usr/bin/g++ g++ /usr/bin/g++-9 && \
    update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-7 70 --slave /usr/bin/g++ g++ /usr/bin/g++-7 && \
    update-alternatives --config gcc


# Check GCC version
RUN gcc --version

# Setup ROS environment
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
SHELL ["/bin/bash", "-c"]
RUN source ~/.bashrc

# Create catkin workspace
RUN mkdir -p ~/catkin_ws/src
WORKDIR /root/catkin_ws/src

# Clone FIESTA repository
RUN git clone https://github.com/hlx1996/Fiesta.git

# Build the workspace
WORKDIR /root/catkin_ws
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# Source the setup file automatically when starting a new shell
RUN echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

# Set the working directory to the source folder for later use
WORKDIR /root/Shared/