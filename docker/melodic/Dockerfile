# syntax=docker/dockerfile:1

FROM osrf/ros:melodic-desktop-full
ARG DEBIAN_FRONTEND=noninteractive
ARG HOME
ARG USER_NAME
ARG UID
ARG GID

SHELL ["/bin/bash", "-c"]

# Install commonly-used tools.
RUN apt-get update && apt-get install --yes \
    curl \
    git \
    iproute2 \
    iputils-ping \
    less \
    mesa-utils \
    net-tools \
    nano \
    rsync \
    software-properties-common \
    sudo \
    tmux \
    tree \
    unzip \
    usbutils \
    vim \
    wget \
    zip \
    zsh


# Install the build tools
RUN add-apt-repository ppa:ubuntu-toolchain-r/test && apt-get update
RUN apt-get update && apt-get install -y \
    build-essential \
    gcc-10 \
    g++-10 \
    clang-format \
    clangd-10 \ 
    jq \
    libsdl-image1.2-dev \
    libopenblas-dev \
    python-catkin-tools \
    python-pip \
    python3-pip

RUN pip install --upgrade pip && pip install cmake
RUN pip install scipy
RUN pip3 install pyyaml rospkg
RUN pip install roslibpy

# Use update alternative to set gcc/g++ to latest version
RUN update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-7 7
RUN update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-10 10
RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-7 7
RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-10 10


# Install the dependecies for CoHAN
RUN apt-get install ros-melodic-navigation -y \
    ros-melodic-pr2-description \
    libqt5widgets5 \
    ros-melodic-mbf-costmap-core \
    libqt5core5a \
    ros-melodic-costmap-converter \
    libzmq3-dev \
    qtbase5-dev \
    libsdl-image1.2-dev \
    libqt5gui5 \
    libsqlite3-dev \
    libyaml-cpp-dev \
    ros-melodic-libg2o \
    ros-melodic-pr2-tuckarm \
    libboost-all-dev \
    ros-melodic-mbf-msgs \
    ros-melodic-teleop-twist-keyboard \
    libsdl2-dev

## Add a non-root user and set proper permission
RUN addgroup --gid $GID $USER_NAME && \
    adduser --uid $UID --gid $GID --disabled-password --gecos "" $USER_NAME

## Add the user to sudo group and remove password requirement
RUN adduser $USER_NAME sudo && echo '%sudo ALL=(ALL) NOPASSWD: ALL' >> /etc/sudoers 

## Change user and copy the appropriate basrc
USER $USER_NAME
COPY ./bashrc /home/$USER_NAME/.bashrc

## Set the workdirectory (Don't forget to mount this in volumes)
WORKDIR $HOME

