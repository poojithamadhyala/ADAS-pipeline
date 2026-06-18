"""
Quick CARLA connectivity test.

Connects to the remote CARLA server (running on RunPod), prints world
info, spawns a vehicle, lets it sit for a few seconds, then cleans up.

Usage:
    python3 test_carla_connection.py <host> <port>

Example:
    python3 test_carla_connection.py 213.173.98.101 28900
"""

import sys
import time
import carla


def main():
    if len(sys.argv) != 3:
        print("Usage: python3 test_carla_connection.py <host> <port>")
        sys.exit(1)

    host = sys.argv[1]
    port = int(sys.argv[2])

    print(f"Connecting to CARLA at {host}:{port} ...")
    client = carla.Client(host, port)
    client.set_timeout(15.0)

    world = client.get_world()
    print(f"Connected. Current map: {world.get_map().name}")

    blueprint_library = world.get_blueprint_library()
    vehicle_bp = blueprint_library.filter("vehicle.tesla.model3")[0]

    spawn_points = world.get_map().get_spawn_points()
    if not spawn_points:
        print("No spawn points found on this map.")
        sys.exit(1)

    spawn_point = spawn_points[0]
    print(f"Spawning vehicle at {spawn_point.location} ...")
    vehicle = world.spawn_actor(vehicle_bp, spawn_point)
    print(f"Vehicle spawned successfully. Actor ID: {vehicle.id}")

    print("Letting it sit for 5 seconds ...")
    time.sleep(5)

    transform = vehicle.get_transform()
    print(f"Vehicle current location: {transform.location}")

    print("Cleaning up (destroying vehicle) ...")
    vehicle.destroy()

    print("Test complete. CARLA server is working correctly.")


if __name__ == "__main__":
    main()
