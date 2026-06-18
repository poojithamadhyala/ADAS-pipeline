# Mac Setup (Phase 1 — no GPU / no CARLA needed)

This phase covers everything that can be built and tested on your Mac:
Docker, ROS Noetic, and the ADAS pipeline container (sensor fusion, ACC,
lane keeping, AEB, vehicle control arbitration). CARLA + the ros-bridge
will be added later on the Windows machine.

## 1. Install Docker Desktop

1. Go to https://www.docker.com/products/docker-desktop/
2. Download the build for your Mac:
   - **Apple Silicon (M1/M2/M3/M4)** -> "Mac with Apple Chip"
   - **Intel Mac** -> "Mac with Intel Chip"
3. Open the `.dmg`, drag Docker to Applications, launch it
4. Wait for the whale icon in the menu bar to show "Docker Desktop is running"

Verify in Terminal:

```bash
docker --version
docker run hello-world
```

## 2. Unzip the project

```bash
unzip adas_dev_env.zip -d adas_dev_env
cd adas_dev_env
```

## 3. Build the ADAS pipeline image

```bash
docker compose -f docker-compose.mac.yml build
```

This builds the `adas-pipeline` image (ROS Noetic + OpenCV + Python deps)
from `docker/adas_pipeline/Dockerfile`. The `roscore` service uses the
official `ros:noetic-ros-core` image and will be pulled automatically.

> **Apple Silicon note:** if no native arm64 image is available, Docker
> will pull the amd64 image and run it under emulation. This works fine
> for ROS/Python nodes like ours, just expect the first `catkin_make`
> build to be a bit slower than on Intel.

## 4. Start roscore + the pipeline

```bash
docker compose -f docker-compose.mac.yml up
```

On first run this will run `catkin_make` inside the container (builds
`adas_msgs` and `adas_pipeline`), then launch all 5 nodes via
`adas_pipeline.launch`. You should see log lines like:

```
[INFO] sensor_fusion_node started
[INFO] acc_node started (target_speed=15.0 m/s, safe_distance=10.0 m)
[INFO] lane_keeping_node started
[INFO] aeb_node started (ttc_threshold=1.50s)
[INFO] vehicle_control_node started
```

`sensor_fusion_node` will sit idle since there's no camera/radar data yet
(that comes from CARLA later) — that's expected.

## 5. Test the ADAS logic manually

Open a second terminal and exec into the running container:

```bash
docker exec -it adas-pipeline bash
source /opt/ros/noetic/setup.bash
```

Publish a fake `ObstacleInfo` message (simulates sensor fusion output):

```bash
rostopic pub /adas/obstacle_info adas_msgs/ObstacleInfo \
  "{distance: 5.0, relative_velocity: -3.0, lane_offset: 0.5, heading_error: 0.1, obstacle_detected: true}"
```

In a third terminal (also `docker exec -it adas-pipeline bash` +
`source /opt/ros/noetic/setup.bash`), watch the outputs:

```bash
rostopic echo /adas/aeb_cmd     # should fire to 1.0 (TTC ~1.67s, close to threshold)
rostopic echo /adas/acc_cmd     # throttle/brake value
rostopic echo /adas/lane_cmd    # steering correction
```

Try different `distance` / `relative_velocity` / `lane_offset` values to
see how each module reacts. This is the full closed-loop logic you'll
later drive with real CARLA sensor data.

## 6. Iterate

- Edit any node under `catkin_ws/src/adas_pipeline/scripts/` on your Mac
  (changes are live via the volume mount)
- Restart the stack to pick up changes:

```bash
docker compose -f docker-compose.mac.yml down
docker compose -f docker-compose.mac.yml up
```

## Next: Phase 2 (Windows + CARLA)

Once the pipeline logic is solid, we'll move to the Windows laptop:
run CARLA natively (using its NVIDIA GPU), run `ros-bridge` +
`adas-pipeline` in WSL2, and connect the two machines so your Mac can
still be used as a dev client if you'd like.
