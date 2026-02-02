#!/usr/bin/env python3
"""
Isaac Sim Navigation Script for RBy1 Robot

This script runs inside Isaac Sim and:
1. Loads the USD scene
2. Enables ROS 2 Bridge
3. Publishes required topics for navigation:
   - /odom (Odometry)
   - /tf (Transforms)
   - /scan or /laser_scan (LiDAR)
   - /clock (Simulation time)

Run with Isaac Sim Python:
    ~/.local/share/ov/pkg/isaac-sim-4.2.0/python.sh run_isaacsim_nav.py --usd /path/to/scene.usd
"""

import sys
import os
import argparse

# Default paths
DEFAULT_USD_PATH = os.path.expanduser(
    "~/isaac_custom_assets/rby1_0813/model_v1.1/model_v1.1.usd"
)


def main():
    parser = argparse.ArgumentParser(description="Isaac Sim Navigation for RBy1")
    parser.add_argument("--usd", type=str, default=DEFAULT_USD_PATH,
                        help="Path to USD scene file")
    parser.add_argument("--headless", action="store_true",
                        help="Run in headless mode")
    args = parser.parse_args()

    # ==================== Initialize Isaac Sim ====================
    from isaacsim import SimulationApp

    CONFIG = {
        "headless": args.headless,
        "renderer": "RayTracedLighting",
        "width": 1920,
        "height": 1080,
        "sync_loads": True,
        "open_usd": args.usd,
    }

    print("=" * 60)
    print("  Isaac Sim + RBy1 Navigation")
    print("=" * 60)
    print(f"USD File: {args.usd}")
    print(f"Headless: {args.headless}")
    print("=" * 60)

    simulation_app = SimulationApp(CONFIG)

    # ==================== Import Omniverse Modules ====================
    import carb
    import numpy as np
    
    import omni
    from omni.isaac.core import World
    from omni.isaac.core.utils.stage import open_stage, get_current_stage
    from omni.isaac.core.utils.extensions import enable_extension
    from omni.isaac.core.utils.prims import get_prim_at_path
    from omni.isaac.core.articulations import Articulation
    
    # Enable ROS 2 Bridge
    enable_extension("omni.isaac.ros2_bridge")
    
    # Wait for extension to load
    simulation_app.update()
    simulation_app.update()

    from omni.isaac.ros2_bridge import read_camera_info
    import omni.graph.core as og

    # ==================== Setup World ====================
    world = World(stage_units_in_meters=1.0)
    
    # Check if USD loaded correctly
    if not os.path.exists(args.usd):
        carb.log_error(f"USD file not found: {args.usd}")
        simulation_app.close()
        sys.exit(1)

    # ==================== Create ROS 2 OmniGraph ====================
    def create_ros2_graph():
        """Create OmniGraph for ROS 2 communication"""
        try:
            # Create a new graph for ROS 2
            keys = og.Controller.Keys
            (graph, nodes, _, _) = og.Controller.edit(
                {"graph_path": "/ROS2_Navigation", "evaluator_name": "execution"},
                {
                    keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        
                        # Clock publisher
                        ("ClockPublisher", "omni.isaac.ros2_bridge.ROS2PublishClock"),
                        
                        # TF publisher
                        ("TFPublisher", "omni.isaac.ros2_bridge.ROS2PublishTransformTree"),
                        
                        # Odometry publisher
                        ("OdomPublisher", "omni.isaac.ros2_bridge.ROS2PublishOdometry"),
                        
                        # LaserScan publisher (if LiDAR exists)
                        ("LaserScanPublisher", "omni.isaac.ros2_bridge.ROS2PublishLaserScan"),
                    ],
                    keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "ClockPublisher.inputs:execIn"),
                        ("OnPlaybackTick.outputs:tick", "TFPublisher.inputs:execIn"),
                        ("OnPlaybackTick.outputs:tick", "OdomPublisher.inputs:execIn"),
                        ("OnPlaybackTick.outputs:tick", "LaserScanPublisher.inputs:execIn"),
                    ],
                    keys.SET_VALUES: [
                        # Clock settings
                        ("ClockPublisher.inputs:topicName", "/clock"),
                        
                        # TF settings
                        ("TFPublisher.inputs:topicName", "/tf"),
                        
                        # Odometry settings
                        ("OdomPublisher.inputs:topicName", "/odom"),
                        ("OdomPublisher.inputs:chassisFrameId", "base_link"),
                        ("OdomPublisher.inputs:odomFrameId", "odom"),
                        
                        # LaserScan settings
                        ("LaserScanPublisher.inputs:topicName", "/scan"),
                        ("LaserScanPublisher.inputs:frameId", "laser_link"),
                    ],
                },
            )
            print("ROS 2 OmniGraph created successfully")
            return graph
        except Exception as e:
            carb.log_warn(f"Could not create full ROS2 graph: {e}")
            carb.log_warn("Some topics may need manual configuration in Isaac Sim")
            return None

    # ==================== Alternative: Use Action Graph from USD ====================
    def setup_ros2_bridge_simple():
        """Simple ROS 2 bridge setup using built-in publishers"""
        try:
            # Enable simulation time
            from omni.isaac.ros2_bridge import _ros2_bridge
            _ros2_bridge.set_use_sim_time(True)
            print("ROS 2 Bridge: Simulation time enabled")
        except Exception as e:
            carb.log_warn(f"Could not enable sim time: {e}")

    # ==================== Main Loop ====================
    print("\nStarting simulation...")
    print("ROS 2 topics available:")
    print("  - /clock (simulation time)")
    print("  - /tf, /tf_static (transforms)")
    print("  - /odom (odometry)")
    print("  - /scan or /laser_scan (LiDAR)")
    print("\nPress Ctrl+C to stop")
    print("=" * 60)

    # Setup ROS 2
    setup_ros2_bridge_simple()
    
    # Try to create the action graph
    create_ros2_graph()

    # Reset and start simulation
    world.reset()
    world.play()

    # Main loop
    step_count = 0
    try:
        while simulation_app.is_running():
            world.step(render=True)
            step_count += 1
            
            # Print status every 1000 steps
            if step_count % 1000 == 0:
                print(f"Simulation step: {step_count}")
                
    except KeyboardInterrupt:
        print("\nShutting down...")

    # Cleanup
    world.stop()
    simulation_app.close()
    print("Isaac Sim closed")


if __name__ == "__main__":
    main()
