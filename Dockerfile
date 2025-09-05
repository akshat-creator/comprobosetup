# ROS 2 Humble (Ubuntu 22.04)
FROM osrf/ros:humble-desktop

# -------- Configurable args --------
ARG USERNAME=ros
ARG UID=1000
ARG GID=1000
ARG TARGETARCH            # set automatically by buildx: amd64 or arm64

# Use bash in RUN steps (so [[ ]] etc. work)
SHELL ["/bin/bash", "-lc"]

# -------- Base tooling --------
RUN apt-get update && apt-get install -y --no-install-recommends \
    locales ca-certificates curl git vim less sudo \
    python3-pip python3-colcon-common-extensions \
    gstreamer1.0-plugins-good gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-tools \
    gstreamer1.0-x gstreamer1.0-alsa gstreamer1.0-gl gstreamer1.0-gtk3 \
    gstreamer1.0-qt5 gstreamer1.0-pulseaudio \
    hping3 \
 && rm -rf /var/lib/apt/lists/*

# -------- Gazebo / Nav2 selection by arch --------
# amd64 (Linux PCs): full Gazebo + Nav2; arm64 (Apple Silicon): skip Gazebo
RUN if [[ "$TARGETARCH" == "amd64" ]]; then \
      apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-gazebo-ros-pkgs \
        ros-humble-navigation2 \
        ros-humble-nav2-bringup \
      && rm -rf /var/lib/apt/lists/*; \
    else \
      echo "Skipping Gazebo on $TARGETARCH"; \
      apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-navigation2 ros-humble-nav2-bringup \
      && rm -rf /var/lib/apt/lists/*; \
    fi

# -------- Non-root user --------
RUN groupadd -g ${GID} ${USERNAME} \
 && useradd -m -s /bin/bash -u ${UID} -g ${GID} ${USERNAME} \
 && usermod -aG sudo ${USERNAME} \
 && echo "%sudo ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/010-sudo-nopasswd

USER ${USERNAME}
WORKDIR /home/${USERNAME}

# -------- Workspace --------
RUN mkdir -p /home/${USERNAME}/ros2_ws/src

# (Optional) fallback clone of neato packages; real dev will bind-mount ./src
RUN cd /home/${USERNAME}/ros2_ws/src && \
    BR=$([[ "$TARGETARCH" == "arm64" ]] && echo no_gazebo || echo main) && \
    git clone -b "$BR" https://github.com/comprobo25/neato_packages.git || true

# -------- Shell setup --------
RUN echo 'source /opt/ros/humble/setup.bash' >> /home/${USERNAME}/.bashrc && \
    echo '[ -f ~/ros2_ws/install/setup.bash ] && source ~/ros2_ws/install/setup.bash' >> /home/${USERNAME}/.bashrc && \
    echo 'export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}' >> /home/${USERNAME}/.bashrc

# hping3 raw socket capability (needed for streaming)
USER root
RUN setcap cap_net_raw+ep /usr/sbin/hping3 || true
USER ${USERNAME}

WORKDIR /home/${USERNAME}/ros2_ws
