# ROS 2 Humble Workspace in Docker

This repository provides a ready-to-use Docker + Compose setup for developing with **ROS 2 Humble** on Ubuntu 22.04 (Jammy).  
It creates an isolated container with your ROS 2 workspace mounted from the host so you can build and run packages without polluting your system.

---

## 🚀 Prerequisites

- [Docker](https://docs.docker.com/get-docker/) installed and running
- [Docker Compose](https://docs.docker.com/compose/) v2+
- Git + SSH configured

---

## 🏗️ Build the Image

From the root of this repo (where the `Dockerfile` and `compose.yaml` are):

```bash
# Build the ROS 2 image locally
docker compose build ros2
```

This will:
- Install ROS 2 Humble desktop + common dev tools
- Set up a user matching your host for file permissions
- Create a catkin/colcon workspace at `/home/<user>/ros2_ws`

---

## ▶️ Run the Container

Start an interactive shell inside the container:

```bash
docker compose run --rm ros2
```

You’ll be dropped into your ROS 2 workspace:

```
/home/<user>/ros2_ws
```

The following directories are mounted from your host:
- `./src` → `/home/<user>/ros2_ws/src`  
- Build, install, and log folders are stored in named volumes:
  - `ros2_build`
  - `ros2_install`
  - `ros2_log`

This means source code stays on your host machine, while build artifacts are isolated to Docker volumes.

---

## 🛠️ Workflow

Typical development cycle inside the container:

```bash
# Clone packages into the workspace
cd ~/ros2_ws/src
git clone git@github.com:comprobo25/neato_packages.git

# Build with colcon
cd ~/ros2_ws
colcon build --symlink-install

# Source setup
source /opt/ros/humble/setup.bash
source install/setup.bash
```

---

## 📝 Notes

- If you hit **permission denied** when editing `src/`, make sure your container user matches your host user.  
  This is already handled in the `Dockerfile` via build args (`USERNAME`, `USER_UID`, `USER_GID`).  
  You can pass them at build time:
  ```bash
  docker compose build --build-arg USERNAME=$USER --build-arg USER_UID=$(id -u) --build-arg USER_GID=$(id -g) ros2
  ```
- SSH keys (`~/.ssh/id_ed25519*`) and git config are mounted for private repo access.
- To keep the container running in the background, you can use:
  ```bash
  docker compose up -d ros2
  docker exec -it ros2-dev bash
  ```

---

## 🧹 Clean Up

To remove the container and volumes:

```bash
docker compose down -v
```

---

## 📚 References

- [ROS 2 Humble Docs](https://docs.ros.org/en/humble/index.html)  
- [Docker Docs](https://docs.docker.com/)  
- [Comprobo Setup](https://comprobo25.github.io/How%20to/setup_your_environment)