"""
Standalone CARLA ADAS Integration (no ROS required)
------------------------------------------------------
Connects directly to a remote CARLA server, spawns an ego vehicle with
a forward-facing radar and camera, then runs simplified ACC, AEB, and
Lane Keeping logic in a plain Python loop -- sending throttle/brake/
steer commands back to the vehicle each tick.

This mirrors the logic from the ROS-based adas_pipeline package
(acc_node.py, aeb_node.py, lane_keeping_node.py, vehicle_control_node.py)
but runs as a single process with no ROS/Docker dependency.

Usage:
    python3 carla_adas_standalone.py <host> <port>

Example:
    python3 carla_adas_standalone.py 213.173.98.101 28900
"""

import sys
import time
import math
import carla


# ---------------------------------------------------------------------
# Tunable parameters (mirrors the ROS node params from Phase 1)
# ---------------------------------------------------------------------
TARGET_SPEED = 15.0       # m/s, ACC cruise target
SAFE_DISTANCE = 10.0      # meters, ACC following distance
TTC_THRESHOLD = 1.5       # seconds, AEB trigger threshold
KP_SPEED = 0.5
KP_DISTANCE = 0.3
KP_LANE_OFFSET = 0.08


class ADASState:
    """Holds the latest sensor readings, updated by sensor callbacks."""

    def __init__(self):
        self.obstacle_distance = float("inf")
        self.obstacle_relative_velocity = 0.0
        self.obstacle_detected = False
        self.lane_offset = 0.0  # placeholder; real lane detection would
        # process camera frames here, as sensor_fusion_node.py did with OpenCV


def radar_callback(state: ADASState, radar_data):
    """Find the closest point ahead from the radar point cloud."""
    closest_depth = float("inf")
    closest_velocity = 0.0
    detected = False

    for detection in radar_data:
        # CARLA radar detections: depth (distance), azimuth, altitude, velocity
        if abs(detection.azimuth) < 0.15 and detection.depth < closest_depth:
            closest_depth = detection.depth
            closest_velocity = detection.velocity
            detected = True

    state.obstacle_detected = detected
    state.obstacle_distance = closest_depth if detected else float("inf")
    state.obstacle_relative_velocity = closest_velocity if detected else 0.0


def compute_acc_throttle(state: ADASState, current_speed: float) -> float:
    """Adaptive Cruise Control: maintain target speed, back off if too close."""
    speed_error = TARGET_SPEED - current_speed
    throttle = KP_SPEED * speed_error

    if state.obstacle_detected:
        distance_error = state.obstacle_distance - SAFE_DISTANCE
        if distance_error < 0:
            brake = KP_DISTANCE * distance_error  # negative
            throttle = min(throttle, brake)

    return max(-1.0, min(1.0, throttle))


def compute_aeb_brake(state: ADASState) -> float:
    """Automatic Emergency Braking: full brake if time-to-collision is too low."""
    if state.obstacle_detected and state.obstacle_relative_velocity < 0:
        ttc = state.obstacle_distance / abs(state.obstacle_relative_velocity)
        if ttc < TTC_THRESHOLD:
            print(f"  [AEB] TRIGGERED - TTC={ttc:.2f}s, distance={state.obstacle_distance:.2f}m")
            return 1.0
    return 0.0


def compute_lane_steering(state: ADASState) -> float:
    """Lane Keeping Assist: simple proportional correction on lane offset."""
    return max(-1.0, min(1.0, -KP_LANE_OFFSET * state.lane_offset))


def main():
    if len(sys.argv) != 3:
        print("Usage: python3 carla_adas_standalone.py <host> <port>")
        sys.exit(1)

    host = sys.argv[1]
    port = int(sys.argv[2])

    print(f"Connecting to CARLA at {host}:{port} ...")
    client = carla.Client(host, port)
    client.set_timeout(15.0)
    world = client.get_world()
    print(f"Connected. Map: {world.get_map().name}")

    blueprint_library = world.get_blueprint_library()

    # --- Spawn ego vehicle ---
    vehicle_bp = blueprint_library.filter("vehicle.tesla.model3")[0]
    spawn_points = world.get_map().get_spawn_points()
    vehicle = world.spawn_actor(vehicle_bp, spawn_points[0])
    print(f"Spawned ego vehicle (id={vehicle.id}) at {spawn_points[0].location}")

    state = ADASState()

    # --- Attach forward radar sensor ---
    radar_bp = blueprint_library.find("sensor.other.radar")
    radar_bp.set_attribute("horizontal_fov", "30")
    radar_bp.set_attribute("vertical_fov", "10")
    radar_bp.set_attribute("range", "50")
    radar_transform = carla.Transform(carla.Location(x=2.0, z=1.0))
    radar = world.spawn_actor(radar_bp, radar_transform, attach_to=vehicle)
    radar.listen(lambda data: radar_callback(state, data))
    print("Radar sensor attached.")

    # --- Spawn a slower lead vehicle ahead to trigger ACC/AEB ---
    lead_bp = blueprint_library.filter("vehicle.audi.a2")[0]
    lead_spawn = spawn_points[1] if len(spawn_points) > 1 else spawn_points[0]
    lead_vehicle = world.spawn_actor(lead_bp, lead_spawn)
    lead_vehicle.set_autopilot(False)
    print(f"Spawned lead vehicle (id={lead_vehicle.id}) for ACC/AEB testing.")

    try:
        print("\nStarting ADAS control loop (Ctrl+C to stop)...\n")
        for tick in range(600):  # ~60 seconds at 10Hz
            velocity = vehicle.get_velocity()
            current_speed = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)

            throttle = compute_acc_throttle(state, current_speed)
            aeb_brake = compute_aeb_brake(state)
            steer = compute_lane_steering(state)

            if aeb_brake >= 1.0:
                final_throttle, final_brake = 0.0, 1.0
            elif throttle >= 0:
                final_throttle, final_brake = throttle, 0.0
            else:
                final_throttle, final_brake = 0.0, -throttle

            control = carla.VehicleControl(
                throttle=final_throttle,
                brake=final_brake,
                steer=steer,
            )
            vehicle.apply_control(control)

            if tick % 10 == 0:
                print(
                    f"[t={tick/10:.1f}s] speed={current_speed:.2f} m/s | "
                    f"obstacle_dist={state.obstacle_distance:.2f}m | "
                    f"throttle={final_throttle:.2f} brake={final_brake:.2f} steer={steer:.2f}"
                )

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nStopped by user.")

    finally:
        print("Cleaning up actors...")
        radar.destroy()
        vehicle.destroy()
        lead_vehicle.destroy()
        print("Done.")


if __name__ == "__main__":
    main()
