# ADAS Dev Environment — Project Summary

## What this project is

A driver-assistance (ADAS) software pipeline built on ROS (Robot
Operating System), designed to integrate with the CARLA autonomous
driving simulator. The pipeline implements four core ADAS modules and
an arbitration layer that combines their outputs into a single vehicle
control command.

## What's built and verified working

### ADAS pipeline (Phase 1 — fully tested)

Five ROS nodes, running in a Docker container, communicating via ROS
topics:

- **Sensor fusion** — combines radar (distance, relative velocity) and
  camera-based lane detection (OpenCV) into a single `ObstacleInfo`
  message
- **Adaptive Cruise Control (ACC)** — maintains a target speed, backs
  off to keep a safe following distance from a detected obstacle
- **Automatic Emergency Braking (AEB)** — computes time-to-collision
  from distance and closing speed; triggers full braking below a
  configurable threshold
- **Lane Keeping Assist (LKA)** — proportional steering correction
  based on lane offset and heading error
- **Vehicle control arbitration** — combines ACC/AEB/LKA outputs into
  a final throttle/brake/steer command, with AEB given override
  priority

This was tested by manually publishing simulated sensor messages
(`rostopic pub`) and confirming each module's output matched expected
calculations — for example, AEB correctly triggered when computed
time-to-collision dropped below the 1.5s threshold, and stayed silent
above it.

### CARLA simulator (Phase 2 — running, confirmed reachable)

CARLA 0.9.15 deployed via Docker on a cloud GPU instance (RunPod, RTX
2000 Ada). Confirmed:

- Container builds and starts cleanly
- Unreal Engine boots successfully (visible in logs)
- The RPC port (2000, mapped externally) accepts raw TCP connections

## What's not yet working: live sensor integration

The remaining step — feeding CARLA's live camera/radar data into the
ADAS pipeline and controlling a simulated vehicle in real time — is
blocked by an environment issue, not a logic issue:

CARLA's Python API client only ships for Linux and Windows (no macOS
build, including no Apple Silicon support), which added friction at
every step of this integration. Beyond that, the CARLA server itself
accepts raw TCP connections but doesn't complete the actual RPC
handshake (`client.get_world()` times out) on the RTX 2000 Ada tier
used for testing, even after multiple clean restarts. This is a known
pattern with CARLA's offscreen rendering mode on certain GPU/driver
combinations — the simulator process starts but its internal
simulation loop doesn't fully initialize.

**Recommended next step:** redeploy the CARLA server on a GPU tier
more commonly validated by the CARLA community (e.g. RTX 3090, RTX
A5000, or similar consumer-class RTX cards rather than workstation/Ada
cards), then retry the connection test with the standalone Python
script below.

## Files

- **`adas_dev_env.zip`** — full ROS package source (the working,
  tested ADAS pipeline), Docker setup for local (Mac) testing
- **`carla_standalone_adas.zip`** — lightweight, ROS-free Python
  script and Dockerfile that connects directly to CARLA via its
  Python API and runs simplified ACC/AEB/LKA logic; this is the
  fastest path to finishing the live integration once a working CARLA
  GPU instance is available
- **`test_carla_connection.py`** — minimal script to verify a CARLA
  server is responding correctly before attempting full integration

## How to resume this project

1. Deploy a new CARLA pod on a consumer RTX GPU tier (RunPod, Paperspace,
   or similar)
2. Run `test_carla_connection.py <host> <port>` from a Linux
   environment (or the provided Docker setup) to confirm
   `client.get_world()` succeeds — this is the key checkpoint that
   was not reached
3. Once confirmed, run `carla_adas_standalone.py <host> <port>` for
   the full closed-loop demo (spawns a vehicle, lead vehicle, and
   runs ACC/AEB/LKA against live sensor data)
4. For the full ROS-based version, the `runpod_ros_bridge.zip` build
   context contains a working Dockerfile (CARLA ROS bridge + ADAS
   pipeline, built from source) — already debugged through several
   issues (GUI package conflicts, Python shebang mismatches) and
   ready to deploy once a working CARLA server is available to point
   it at
