#!/usr/bin/env python3
"""
Isaac Sim standalone script for RBy1 Robot with ROS 2 bridge
This script should be run using Isaac Sim's Python interpreter:
    ~/.local/share/ov/pkg/isaac-sim-4.2.0/python.sh run_isaacsim.py
"""

import sys
import os
import argparse

# Default paths
DEFAULT_USD_PATH = os.path.expanduser("~/isaac_custom_assets/rby1_0813/model_v1.1/model_v1.1.usd")

def main():
    parser = argparse.ArgumentParser(description="Launch Isaac Sim with RBy1 robot")
    parser.add_argument("--usd", type=str, default=DEFAULT_USD_PATH,
                        help="Path to USD file")
    parser.add_argument("--headless", action="store_true",
                        help="Run in headless mode")
    args = parser.parse_args()

    # Import Isaac Sim modules (must be done after isaacsim is initialized)
    from isaacsim import SimulationApp

    # Launch configuration
    CONFIG = {
        "headless": args.headless,
        "renderer": "RayTracedLighting",
        "width": 1920,
        "height": 1080,
    }

    # Create simulation app
    simulation_app = SimulationApp(CONFIG)

    # Now import Omniverse modules
    import carb
    from omni.isaac.core import World
    from omni.isaac.core.utils.stage import open_stage
    import omni.isaac.ros2_bridge as ros2_bridge

    print("=" * 50)
    print("  Isaac Sim + RBy1 Robot + ROS 2")
    print("=" * 50)

    # Check if USD file exists
    if not os.path.exists(args.usd):
        carb.log_error(f"USD file not found: {args.usd}")
        simulation_app.close()
        sys.exit(1)

    print(f"Loading USD: {args.usd}")

    # Open the stage
    open_stage(args.usd)

    # Create world
    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()

    # Enable ROS 2 bridge extension
    from omni.isaac.core.utils.extensions import enable_extension
    enable_extension("omni.isaac.ros2_bridge")

    print("ROS 2 Bridge enabled")
    print("Simulation starting...")

    # Reset world
    world.reset()

    # Main simulation loop
    while simulation_app.is_running():
        world.step(render=True)

    # Cleanup
    simulation_app.close()

if __name__ == "__main__":
    main()
