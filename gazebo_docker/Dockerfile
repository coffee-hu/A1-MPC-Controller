# we build this docker based on a docker with ros already installed
# FROM osrf/ros:melodic-desktop-full

FROM nvidia/opengl:1.0-glvnd-devel-ubuntu18.04

# This avoids keyboard interaction when asked for geographic area
ARG DEBIAN_FRONTEND=noninteractive

# Change apt source to ustc
RUN mv /etc/apt/sources.list /etc/apt/sources_backup.list && \
    echo "deb http://mirrors.ustc.edu.cn/ubuntu/ bionic main restricted universe multiverse" >> /etc/apt/sources.list && \
    echo "deb http://mirrors.ustc.edu.cn/ubuntu/ bionic-updates main restricted universe multiverse" >> /etc/apt/sources.list && \
    echo "deb http://mirrors.ustc.edu.cn/ubuntu/ bionic-backports main restricted universe multiverse" >> /etc/apt/sources.list && \
    echo "deb http://mirrors.ustc.edu.cn/ubuntu/ bionic-security main restricted universe multiverse" >> /etc/apt/sources.list

RUN apt-get update && apt-get install -y sudo

RUN apt-get update && \
    apt-get install -y \
    build-essential \
    curl \
    doxygen \
    git \
    gnupg2 \
    libbluetooth-dev \
    libcwiid-dev \
    libgoogle-glog-dev \
    libspnav-dev \
    libusb-dev \
    locales \
    lsb-release \
    mercurial \
    python3-dbg \
    python3-empy \
    python3-numpy \
    python3-pip \
    python3-psutil \
    python3-venv \
    software-properties-common \
    sudo \
    tzdata \
    vim \
    rsync \
    ufw \
    gdb \
    gdbserver \
    wget \
    openssh-client \
    clang-format \
    tmux \
    openssh-server \
    bash-completion \
    && apt-get clean

ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
#Add ROS key
RUN sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-melodic.list'


RUN export DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
    && apt-get install -y \
    python3-rosdep \
    python3-vcstool \
    python3-argcomplete \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    ros-melodic-desktop-full\
    && apt-get clean

LABEL maintainer="shuoyang@andrew.cmu.edu"
ENV REFRESH_AT 2022-03-22

# nvidia-container-runtime
# ENV NVIDIA_VISIBLE_DEVICES \
#     ${NVIDIA_VISIBLE_DEVICES:-all}
# ENV NVIDIA_DRIVER_CAPABILITIES \
#     ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# install necessary dependencies 
RUN   if [ "x$(nproc)" = "x1" ] ; then export USE_PROC=1 ; \
      else export USE_PROC=$(($(nproc)/2)) ; fi && \
      apt-get update && apt-get install -y \
      vim \
      libatlas-base-dev \
      libeigen3-dev \
      libsuitesparse-dev \
      python-catkin-tools \
      python3-matplotlib \
      gfortran \
      autoconf \
      coinor-libipopt-dev \
      libgfortran3 \
      libopenmpi-dev \
      apt-utils \
      libssl-dev \
      libtool 

# install python 3.6 
RUN apt-get update && \
    apt-get install -y python3.6
RUN update-alternatives --install /usr/bin/python python /usr/bin/python3.6 1
RUN curl https://bootstrap.pypa.io/pip/3.6/get-pip.py -o get-pip.py
RUN python get-pip.py
RUN rm get-pip.py

RUN pip --version
RUN pip install pybullet==3.1.7
RUN pip install tensorflow==1.15.4
RUN pip install numpy==1.17.3 -i https://pypi.tuna.tsinghua.edu.cn/simple
RUN pip install scipy -i https://pypi.tuna.tsinghua.edu.cn/simple


# install cmake 3.20 
WORKDIR /tmp 
RUN apt-get -y install wget libtool
RUN wget https://github.com/Kitware/CMake/releases/download/v3.20.2/cmake-3.20.2.tar.gz
RUN tar -zxvf cmake-3.20.2.tar.gz
RUN ls
WORKDIR /tmp/cmake-3.20.2
RUN ./bootstrap
RUN make
RUN make install

# change timezone (this is very important otherwise many ROS topic time will be strange)
ENV TZ=America/New_York
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone
# add OSQP 
# follow https://osqp.org/docs/get_started/sources.html#build-from-sources to install OSQP from sources
WORKDIR /tmp
RUN git clone --recursive -b release-0.6.3 https://github.com/oxfordcontrol/osqp
WORKDIR /tmp/osqp
RUN mkdir build 
WORKDIR /tmp/osqp/build
RUN cmake -G "Unix Makefiles" ..
RUN cmake --build .
RUN cmake --build . --target install

# add OSQP-python
# RUN pip install osqp
RUN pip install osqp -i https://pypi.tuna.tsinghua.edu.cn/simple
RUN apt-get install -y ipython

# add osqp-eigen
WORKDIR /tmp
RUN git clone https://github.com/robotology/osqp-eigen.git
WORKDIR /tmp/osqp-eigen
RUN mkdir build 
WORKDIR /tmp/osqp-eigen/build
RUN cmake ../
RUN make
RUN make install

# make unitree workspace
ENV SUPPORT_WS=/root/support_files
ENV UNITREE_WS=/root/unitree_ws
RUN mkdir -p $SUPPORT_WS 
RUN mkdir -p $UNITREE_WS/src
WORKDIR $UNITREE_WS
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash; catkin init;"

# install unitree sdk dependencies
WORKDIR $SUPPORT_WS
RUN git clone https://github.com/lcm-proj/lcm.git && \
    cd ${SUPPORT_WS}/lcm && \
    git checkout tags/v1.4.0 && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make -j12 && \
    make install 

# notice we must use v3.2 
WORKDIR $SUPPORT_WS
RUN git clone https://github.com/unitreerobotics/unitree_legged_sdk.git && \
    cd ${SUPPORT_WS}/unitree_legged_sdk && git checkout v3.2 && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make -j12 

WORKDIR $SUPPORT_WS
RUN git clone https://github.com/unitreerobotics/aliengo_sdk.git && \
    cd ${SUPPORT_WS}/aliengo_sdk && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make -j12     

# install necessary dependencies 
RUN   if [ "x$(nproc)" = "x1" ] ; then export USE_PROC=1 ; \
      else export USE_PROC=$(($(nproc)/2)) ; fi && \
      apt-get update && apt-get install -y \
      ros-melodic-ros-control \
      ros-melodic-gazebo-ros \
      ros-melodic-joy \
      ros-melodic-ros-controllers \
      ros-melodic-robot-state-publisher \
      ros-melodic-plotjuggler-ros

WORKDIR $UNITREE_WS/src
RUN git clone https://github.com/ShuoYangRobotics/unitree_ros.git
WORKDIR $UNITREE_WS

ENV UNITREE_SDK_VERSION=3_2
ENV UNITREE_LEGGED_SDK_PATH=${SUPPORT_WS}/unitree_legged_sdk
ENV ALIENGO_SDK_PATH=${SUPPORT_WS}/aliengo_sdk
ENV UNITREE_PLATFORM=amd64
RUN echo "#unitree config" >> ~/.bashrc
RUN echo "export export UNITREE_SDK_VERSION=3_2" >> ~/.bashrc
RUN echo "export UNITREE_LEGGED_SDK_PATH=${SUPPORT_WS}/unitree_legged_sdk" >> ~/.bashrc
RUN echo "export ALIENGO_SDK_PATH=${SUPPORT_WS}/aliengo_sdk" >> ~/.bashrc
RUN echo "export UNITREE_PLATFORM=\"amd64\"" >> ~/.bashrc
RUN echo "source ${UNITREE_WS}/devel/setup.bash" >> ~/.bashrc

# compile unitree ros 
RUN ls $UNITREE_WS/src/unitree_ros
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash; catkin build unitree_legged_msgs; source ${UNITREE_WS}/devel/setup.bash; catkin build"


RUN echo "export ROS_MASTER_URI=http://localhost:11311;export ROS_IP=localhost;export ROS_HOSTNAME=localhost" >> ~/.bashrc

# To use rosparam load yaml files
RUN pip install pyyaml -i https://pypi.tuna.tsinghua.edu.cn/simple

# default use python 2.7, or type this command when docker starts 
RUN update-alternatives --install /usr/bin/python python /usr/bin/python2.7 3