# Ubuntu 22.04 (Jammy) + ROS 2 Humble desktop-full
FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

# Locale (UTF-8)
RUN apt-get update && apt-get install -y locales \
 && locale-gen en_US en_US.UTF-8 \
 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
 && rm -rf /var/lib/apt/lists/*
ENV LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8

# Dev tools for ROS workspaces
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    git \
    wget \
    curl \
    python3-pip \
    python3-colcon-common-extensions \
    python3-vcstool \
    python3-rosdep \
    sudo \
    less \
    vim \
 && rm -rf /var/lib/apt/lists/*

# ROS + sim + streaming deps (you already had these)
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-nav2-bringup \
    ros-humble-navigation2 \
    ros-humble-camera-info-manager \
    ros-humble-cartographer-ros \
    ros-humble-cartographer \
    ros-humble-gscam \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav \
    gstreamer1.0-tools \
    gstreamer1.0-x \
    gstreamer1.0-alsa \
    gstreamer1.0-gl \
    gstreamer1.0-gtk3 \
    gstreamer1.0-qt5 \
    gstreamer1.0-pulseaudio \
    hping3 \
    # helpful runtime libs for OpenCV GUI backends
    libgl1 \
    libglib2.0-0 \
 && rm -rf /var/lib/apt/lists/*

# CUDA container hints (optional)
ENV NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=all,compute,utility,graphics

# rosdep init/update (ignore if already initialized)
RUN rosdep init || true && rosdep update || true

# Create a non-root developer user
ARG USERNAME=ros
ARG UID=1000
ARG GID=1000
RUN groupadd -g ${GID} ${USERNAME} \
 && useradd -m -s /bin/bash -u ${UID} -g ${GID} ${USERNAME} \
 && usermod -aG sudo ${USERNAME} \
 && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/90-${USERNAME}

# ---------- Project setup done as root so we can install deps system-wide ----------
# Install Python packages system-wide (wheel-based; no cache to keep image slim)
RUN python3 -m pip install --no-cache-dir --upgrade pip \
 && python3 -m pip install --no-cache-dir scikit-build opencv-python

# Allow raw sockets for hping3 so video streaming works
RUN setcap cap_net_raw+ep /usr/sbin/hping3 || true

# Workspace + Neato packages
ARG NEATO_BRANCH=main
ARG NEATO_REPO=https://github.com/comprobo25/neato_packages.git
RUN mkdir -p /home/${USERNAME}/ros2_ws/src \
 && git clone -b ${NEATO_BRANCH} ${NEATO_REPO} /home/${USERNAME}/ros2_ws/src/neato_packages

# Resolve package dependencies (apt) for the workspace
RUN bash -lc "source /opt/ros/humble/setup.bash && \
              rosdep install --from-paths /home/${USERNAME}/ros2_ws/src --ignore-src -r -y"

# Make sure the normal user owns the workspace
RUN chown -R ${USERNAME}:${GID} /home/${USERNAME}/ros2_ws

# ---------- Switch to non-root for building and running ----------
USER ${USERNAME}
WORKDIR /home/${USERNAME}

# Build the workspace (colcon) and set up auto-sourcing
RUN bash -lc "source /opt/ros/humble/setup.bash && \
              cd ~/ros2_ws && colcon build --symlink-install"

# Auto-source ROS and workspace on new shells
RUN echo 'source /opt/ros/humble/setup.bash' >> /home/${USERNAME}/.bashrc \
 && echo '[ -f ~/ros2_ws/install/setup.bash ] && source ~/ros2_ws/install/setup.bash' >> /home/${USERNAME}/.bashrc

ARG ROS_DOMAIN_ID=4
ENV ROS_DOMAIN_ID=${ROS_DOMAIN_ID}
RUN echo "export ROS_DOMAIN_ID=${ROS_DOMAIN_ID}" >> /home/${USERNAME}/.bashrc

CMD ["/bin/bash"]
