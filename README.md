# InternNav-ros2-client

Real-time robot control workspace for InternNav. Runs on the Unitree Go2 robot and consumes server outputs to generate low-level velocity commands at 100 Hz.

## 🔧 Prerequisites

### zenoh-bridge-ros2dds

Install and run zenoh-bridge-ros2dds on both the server and client machines.
Refer to the [official documentation](https://zenoh.io/docs/getting-started/installation/).
> The version must match on both the server and client. 

```bash
echo '
TODO!!!
' >> <path to zenoh config>
```
>  Zenoh config file should be a '.json5' format

---

## 📦 Package

| Package | Build Type | Description |
|---------|------------|-------------|
| `internnav_client` | ament_python | Planner and Controller nodes |

---

## 🤖 Nodes

### `internnav_planner` — Path Transformation & Action Dispatch

Receives outputs from both server nodes and prepares them for the controller.

**Parameters**

| Name | Type | Default | Description |
|------|------|---------|-------------|
| `slowdown_factor` | int | `1` | Linear interpolation factor for trajectory smoothing |
| `rotation_degree` | int | `15` | Yaw increment (degrees) per TURN_LEFT / TURN_RIGHT action |

**Subscribed Topics**

| Topic | Type | Description |
|-------|------|-------------|
| `/utlidar/robot_odom` | `nav_msgs/Odometry` | Robot odometry for frame transforms |
| `/internnav/server/system1/output_path` | `nav_msgs/Path` | Trajectory in `base_footprint` frame from System1 |
| `/internnav/server/system2/output_discretes` | `internnav_interfaces/DiscreteStamped` | Discrete actions from System2 |

**Published Topics**

| Topic | Type | Description |
|-------|------|-------------|
| `/internnav/client/cmd_path` | `nav_msgs/Path` | Trajectory in world (`odom`) frame → triggers MPC mode |
| `/internnav/client/cmd_pose` | `geometry_msgs/PoseStamped` | Goal pose from discrete action → triggers PD mode |
| `/internnav/client/cmd_stop` | `std_msgs/Header` | Stops the robot immediately |

**Discrete action processing (System2 output)**

| Action | Behavior |
|--------|----------|
| `STOP` | Publishes `cmd_stop` immediately |
| `FORWARD` | Accumulates 0.25 m step along current heading, publishes goal as `cmd_pose` |
| `TURN_LEFT` / `TURN_RIGHT` | Accumulates 15° rotation, publishes goal as `cmd_pose` |
| `LOOK_DOWN` | No movement — skipped |

---

### `internnav_controller` — Real-time Motion Control

Runs a 100 Hz feedback loop and issues velocity commands to the Unitree Go2 via the sport API.

**Parameters**

| Name | Type | Default | Description |
|------|------|---------|-------------|
| `hz` | float | `100` | Control loop frequency |

**Subscribed Topics**

| Topic | Type | Description |
|-------|------|-------------|
| `/utlidar/robot_odom` | `nav_msgs/Odometry` | Current robot state |
| `/internnav/client/cmd_path` | `nav_msgs/Path` | Trajectory command → switches to MPC mode |
| `/internnav/client/cmd_pose` | `geometry_msgs/PoseStamped` | Goal pose → switches to PD mode |
| `/internnav/client/cmd_stop` | `std_msgs/Header` | Switches to IDLE mode immediately |

**Published Topics**

| Topic | Type | Description |
|-------|------|-------------|
| `/api/sport/request` | `unitree_api/Request` | JSON-encoded sport API command (API ID 1008) |

**Control modes**

| Mode | Trigger | Controller | Description |
|------|---------|------------|-------------|
| `IDLE` | `cmd_stop` | — | No output |
| `MPC` | `cmd_path` | `MPCController` | Trajectory tracking |
| `PD` | `cmd_pose` | `PDController` | Executes a single short-step primitive from a discrete action |

---

## 🛠️ Build

```bash
mkdir -p InternNav_ws/src
cd InternNav_ws/src

# 1. Pull Unitree_ros2
git init
git remote add -f origin https://github.com/unitreerobotics/unitree_ros2
git config core.sparseCheckout true
echo "cyclonedds_ws/*" >> .git/info/sparse-checkout
git pull origin master

# 2. Clone InternNav-ros2 repositories
git clone https://github.com/yubincho3/InternNav-ros2-interfaces
git clone https://github.com/yubincho3/InternNav-ros2-client

# 3. Setup venv
cd ..
python3 -m venv .venv
source .venv/bin/activate

# 4. Install build tools (colcon)
pip install colcon-common-extensions catkin_pkg "empy<4" lark

# 5. Install dependencies
pip install -r src/InternNav-ros2-client/requirements.txt

# 6. Build packages
source /opt/ros/<distro>/setup.bash
colcon build --symlink-install
```



**External dependencies**

| Dependency | Purpose |
|------------|---------|
| `casadi` | MPC optimization |
| `unitree_ros2` | Sport API request message type |
| `numpy`, `scipy` | Math utilities |

---

## 🚀 Launch

```bash
# Terminal 1. Turn on zenoh bridge
zenoh-bridge-ros2dds -c <path to zenoh config>

# Terminal 2. Launch InternNav client
source /opt/ros/<distro>/setup.bash
source install/setup.bash
ros2 launch internnav_client realworld.launch.py
```

```bash
#### Example ####
# Terminal 1. Turn on zenoh bridge
zenoh-bridge-ros2dds -c zenoh-config.json5

# Terminal 2. Launch client
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 launch internnav_client realworld.launch.py
```

Launches both `internnav_controller` and `internnav_planner` with output to screen.  
No additional arguments are required; parameters can be overridden with `param_file` or via `ros2 param set` at runtime.

## 👏 Acknowledgements

This project is based on [InternNav](https://github.com/InternRobotics/InternNav) by Intern Robotics.
The original codebase has been adapted from an HTTP/multi-threaded architecture to a ROS 2 architecture for real-world deployment on the Unitree Go2.

## 📄 License

This project is licensed under the Apache 2.0 License. See [LICENSE](LICENSE) for details.
