# 🤖 ROS 2 Humble Workspace in Docker

This repository provides a ready-to-use **Docker + Compose** setup for developing with **ROS 2 Humble** on Ubuntu 22.04 (Jammy).  
It creates an isolated container with your ROS 2 workspace mounted from the host so you can build and run packages without polluting your system.


## 📦 Clone This Repository

Clone with SSH:
```bash
git clone git@github.com:akshat-creator/comprobosetup.git
```


## 🚀 Prerequisites

* [Docker](https://docs.docker.com/get-docker/) installed and running
* [Docker Compose](https://docs.docker.com/compose/) v2+
* Git + SSH configured (for cloning private repositories)

## Set Your ROS_DOMAIN_ID

Before running the container, students should manually edit the `ROS_DOMAIN_ID` in the `compose.yaml` file to their assigned number. By default, it is set to `0`.


## 🖥️ Architecture Note

Gazebo is **not supported** on Apple Silicon (macOS) . The current Dockerfile recognizes your architecture and skips the gazebo package.

If you need Gazebo, you must run on an amd64 (Linux PC) machine. Navigation2 works on both architectures.


## 🏗️ Build the Image

Pull the prebuilt image (works for both macOS + Linux):

```bash
docker pull redhotchili/olin-comprobo-ros2:humble
```

Or build it locally yourself from the root of this repo:

```bash
docker compose build \
  --build-arg USERNAME=$USER \
  --build-arg UID=$(id -u) \
  --build-arg GID=$(id -g) \
  ros2
```

## ▶️ Run the Container

Start an interactive shell inside the container:

```bash
docker compose run --rm ros2
```

Your ROS 2 workspace will be at:

```
/home/ros/ros2_ws
```
Fix volume ownership by running this once: 
```
sudo mkdir -p ~/ros2_ws/{build,install} ~/.ros 
sudo chown -R $(id -u):$(id -g) ~/ros2_ws/{build,install} ~/.ros 
```
This ensures you can write into the mounted volumes.

## 🛠️ Workflow

Typical development cycle inside the container:

```bash
cd ~/ros2_ws/src
git clone git@github.com:comprobo25/neato_packages.git
cd ~/ros2_ws
colcon build --symlink-install
source /opt/ros/humble/setup.bash
source install/setup.bash
```


## ✅ Quick Test

After entering the container and sourcing ROS:

```bash
ros2 run demo_nodes_cpp talker
```

In another terminal:

```bash
docker compose run --rm ros2
ros2 topic echo /chatter
```

If you see messages streaming, your environment is working!


## 🔑 Git & SSH Setup

This compose mounts your host identity **read-only into `/cache`** and copies it into `~/.ssh` at startup so permissions are correct:

- `${HOME}/.gitconfig` → `/cache/git/.gitconfig` → copied to `~/.gitconfig`
- `${HOME}/.ssh/id_ed25519*` → `/cache/ssh/…` → copied to `~/.ssh/…`
- `${HOME}/.ssh/known_hosts` → `/cache/ssh/known_hosts` → `~/.ssh/known_hosts`


## 📝 Notes

* If you see **Permission denied** in `src/`, ensure your container user matches your host user.
  Use the build args (`USERNAME`, `UID`, `GID`) to match:

  ```bash
  docker compose build \
    --build-arg USERNAME=$USER \
    --build-arg UID=$(id -u) \
    --build-arg GID=$(id -g) \
    ros2
  ```

* If you hit **Permission denied** under `build/` or `install/`, run once inside the container:
  ```bash
  sudo mkdir -p ~/ros2_ws/{build,install} ~/.ros
  sudo chown -R $(id -u):$(id -g) ~/ros2_ws/{build,install} ~/.ros
  ```

* To keep the container running in background:

  ```bash
  docker compose up -d ros2
  docker exec -it ros2-dev bash
  ```

## 🧹 Clean Up

  To remove the container and volumes:

  ```bash
  docker compose down -v
  ```


## 📚 References

* [ROS 2 Humble Docs](https://docs.ros.org/en/humble/index.html)
* [Docker Docs](https://docs.docker.com/)
* [Comprobo Setup](https://comprobo25.github.io/How%20to/setup_your_environment)


