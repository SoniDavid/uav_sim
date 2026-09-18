FROM osrf/ros:humble-desktop-full

# Add a non-root user
ARG USERNAME=dev
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Install common dev tools, Foxglove bridge, and prerequisites for Gazebo
RUN apt-get update && apt-get install -y \
    python3-pip \
    git \
    wget \
    lsb-release \
    gnupg \
    ros-humble-foxglove-bridge \
    && rm -rf /var/lib/apt/lists/*

# Add OSRF repository and install Gazebo Harmonic
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
    && apt-get update \
    && apt-get install -y \
    gz-harmonic \
    && rm -rf /var/lib/apt/lists/*

# Install MuJoCo 3.1.6 manually since libmujoco-dev is not available on Jammy (Ubuntu 22.04)
# We also install dependencies for rendering and development
RUN apt-get update && apt-get install -y \
    libgl1-mesa-dev \
    libosmesa6-dev \
    libglew-dev \
    libglfw3-dev \
    && rm -rf /var/lib/apt/lists/* \
    && mkdir -p /opt/mujoco \
    && wget https://github.com/google-deepmind/mujoco/releases/download/3.1.6/mujoco-3.1.6-linux-x86_64.tar.gz -O mujoco.tar.gz \
    && tar -xf mujoco.tar.gz -C /opt/mujoco --strip-components=1 \
    && rm mujoco.tar.gz

ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/mujoco/lib
ENV CPATH=$CPATH:/opt/mujoco/include
ENV PATH=$PATH:/opt/mujoco/bin

USER $USERNAME
ENV SHELL /bin/bash
