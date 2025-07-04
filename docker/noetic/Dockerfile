# syntax=docker/dockerfile:1

FROM osrf/ros:noetic-desktop-full
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
    clang-format \
    clangd-10 \ 
    jq \
    gcc-10 \
    g++-10 \
    libsdl-image1.2-dev \
    libopenblas-dev \
    python3-catkin-tools \
    python3-pip \
    python-is-python3 

RUN pip install --upgrade pip && pip install cmake==3.31.1
RUN pip install scipy

# Use update alternative to set gcc/g++ to latest version
RUN update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-9 9
RUN update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-10 10
RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-9 9
RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-10 10


# Install the dependecies for CoHAN
RUN apt-get install ros-noetic-navigation -y \
    ros-noetic-pr2-tuckarm \
    ros-noetic-costmap-converter \
    ros-noetic-libg2o \
    ros-noetic-mbf-costmap-core \
    ros-noetic-mbf-msgs \
    libsqlite3-dev \
    libzmq3-dev \
    qtbase5-dev \
    libqt5core5a \
    libqt5gui5 \
    libqt5widgets5 \
    libsdl-image1.2-dev \
    libyaml-cpp-dev \
    libboost-all-dev \
    ros-noetic-pr2-description \
    ros-noetic-teleop-twist-keyboard \
    ros-noetic-pr2-simulator \
    ros-noetic-pr2-gazebo \
    ros-noetic-teb-local-planner \
    libsdl2-dev

# Add Webots repository and install Webots
RUN wget https://github.com/cyberbotics/webots/releases/download/R2023a/webots-R2023a-x86-64.tar.bz2 && \
    tar -xjf webots-R2023a-x86-64.tar.bz2 && \
    mv webots /usr/local/webots && \
    rm webots-R2023a-x86-64.tar.bz2

ENV PATH="/usr/local/webots:${PATH}"

RUN apt-get update && apt-get install -y libsndio7.0
RUN ln -s /usr/lib/x86_64-linux-gnu/libsndio.so.7.0 /usr/lib/x86_64-linux-gnu/libsndio.so.7

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


