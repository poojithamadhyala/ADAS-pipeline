# ADAS Dev Environment (Docker + ROS + CARLA)

A containerized development environment for prototyping and integrating
Advanced Driver-Assistance System (ADAS) modules against the CARLA
simulator, using ROS (Noetic) as the message-passing backbone.

## Architecture

```
                    +----------------------+
                    |   CARLA Simulator    |
                    |  (Town01, ego car)   |
                    +----------+-----------+
                               |
                       CARLA <-> ROS
                               |
                    +----------v-----------+
                    |     ros-bridge        |
                    | (camera, radar, lidar |
                    |  vehicle status/ctrl) |
                    +----------+-----------+
                               |
                ----------------------------------
                |                                |
       +--------v---------+            +---------v--------+
       | sensor_fusion_node|            | vehicle_status    |
       | (camera + radar)  |            +---------+--------+
       +--------+---------+                      |
                |  /adas/obstacle_info             |
     -----------+-----------+----------+          |
     |                       |          |          |
+----v----+           +------v----+ +---v------+   |
| acc_node|           | lane_keep | | aeb_node |   |
+----+----+           +-----+-----+ +----+-----+   |
     | /adas/acc_cmd        | /adas/lane_cmd  | /adas/aeb_cmd
     |                       |                |
     +-----------+-----------+----------------+
                  |
        +---------v----------+
        | vehicle_control_node|
        | (arbitration)       |
        +---------+-----------+
                  |
       /carla/ego_vehicle/vehicle_control_cmd
                  |
                  v
              CARLA car
```

## ADAS modules included

| Module | Node | Function |
|---|---|---|
| Sensor Fusion | `sensor_fusion_node.py` | Fuses radar (distance/relative velocity) and camera (lane offset/heading) into one `ObstacleInfo` message |
| Adaptive Cruise Control | `acc_node.py` | Maintains target speed; backs off to keep a safe following distance |
| Lane Keeping Assist | `lane_keeping_node.py` | PD steering controller to keep the car centered in its lane |
| Automatic Emergency Braking | `aeb_node.py` | Computes time-to-collision; triggers full brake if below threshold |
| Vehicle Control | `vehicle_control_node.py` | Arbitrates between AEB/ACC/LKA outputs and sends the final command to CARLA |

## Prerequisites

- A host machine with an **NVIDIA GPU** + drivers + the
  [NVIDIA Container Toolkit](https://github.com/NVIDIA/nvidia-container-toolkit)
  (CARLA needs GPU rendering, even with `-RenderOffScreen`)
- Docker Engine + Docker Compose v2
- ~30 GB free disk space for the CARLA image

> **Note on VirtualBox:** VirtualBox does not support GPU passthrough well
> enough to run CARLA at usable framerates. If you don't have a Linux host
> with an NVIDIA GPU, run Docker directly on a GPU-equipped Linux machine
> instead. VirtualBox is still useful if you just want an isolated Ubuntu
> dev VM to install Docker + the NVIDIA toolkit inside.

## Quick start

```bash
# 1. Build and start everything (CARLA, ROS bridge, ADAS pipeline)
docker compose up --build

# 2. In another terminal, check that topics are flowing
docker exec -it carla-ros-bridge bash
source /opt/ros/noetic/setup.bash
rostopic list
rostopic hz /adas/obstacle_info
```

The `adas-pipeline` container runs `catkin_make` on first start (the
`catkin_ws` directory is mounted as a volume, so you can edit Python
files on your host and just restart the container to pick up changes).

## Topic reference

| Topic | Type | Published by | Consumed by |
|---|---|---|---|
| `/carla/ego_vehicle/radar_front` | `sensor_msgs/PointCloud2` | ros-bridge | sensor_fusion_node |
| `/carla/ego_vehicle/camera_front/image_color` | `sensor_msgs/Image` | ros-bridge | sensor_fusion_node |
| `/adas/obstacle_info` | `adas_msgs/ObstacleInfo` | sensor_fusion_node | acc_node, lane_keeping_node, aeb_node |
| `/adas/acc_cmd` | `std_msgs/Float32` | acc_node | vehicle_control_node |
| `/adas/lane_cmd` | `std_msgs/Float32` | lane_keeping_node | vehicle_control_node |
| `/adas/aeb_cmd` | `std_msgs/Float32` | aeb_node | vehicle_control_node |
| `/carla/ego_vehicle/vehicle_control_cmd` | `carla_msgs/CarlaEgoVehicleControl` | vehicle_control_node | ros-bridge |

## Testing without CARLA

You don't need CARLA running to test the ADAS logic itself. Start only
the `adas-pipeline` container and publish fake `ObstacleInfo` messages
by hand:

```bash
rostopic pub /adas/obstacle_info adas_msgs/ObstacleInfo \
  "{distance: 5.0, relative_velocity: -3.0, lane_offset: 0.5, heading_error: 0.1, obstacle_detected: true}"
```

Then watch the AEB/ACC/LKA outputs:

```bash
rostopic echo /adas/aeb_cmd
rostopic echo /adas/acc_cmd
rostopic echo /adas/lane_cmd
```

With the values above, AEB should trigger (TTC = 5.0 / 3.0 ≈ 1.67s is
close to the 1.5s threshold — try `relative_velocity: -5.0` to force it).

## Extending the pipeline

To add a new ADAS module (e.g. Blind Spot Detection):

1. Add a new script under `catkin_ws/src/adas_pipeline/scripts/`
2. Register it in `CMakeLists.txt` under `catkin_install_python`
3. Add a `<node>` entry to `launch/adas_pipeline.launch`
4. If it needs a new message type, define it in
   `catkin_ws/src/adas_msgs/msg/` and add it to `adas_msgs/CMakeLists.txt`

## Tuning parameters

Key tunables are exposed as ROS params in `adas_pipeline.launch`:

- `acc_node`: `target_speed` (m/s), `safe_distance` (m)
- `aeb_node`: `ttc_threshold` (seconds)

Controller gains (`kp_speed`, `kp_distance`, `kp_offset`, `kd_heading`)
are currently hardcoded in each node — pull these out as params too if
you want to tune them without rebuilding.
