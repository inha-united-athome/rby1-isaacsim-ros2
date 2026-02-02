#!/usr/bin/env python3
"""
Script to load USD file and start simulation in Isaac Sim
This script is executed inside Isaac Sim via --exec option
Creates a signal file when ready for Nav2 to start
"""
import os
import sys
import carb
import omni.kit.app
import omni.usd
import omni.timeline
import asyncio

# USD file path - from environment variable or default
usd_path = os.environ.get("ISAAC_USD_PATH", os.path.expanduser("~/isaac_custom_assets/0926Final.usd"))

# Signal file to indicate Isaac Sim is ready
signal_file = "/tmp/isaac_sim_ready"

# Remove old signal file if exists
if os.path.exists(signal_file):
    os.remove(signal_file)

print(f"[load_usd_and_play] Will load USD file: {usd_path}", flush=True)

async def load_and_play():
    # Wait for the app to be ready
    for _ in range(10):
        await omni.kit.app.get_app().next_update_async()
    
    print(f"[load_usd_and_play] Opening stage: {usd_path}", flush=True)
    
    # Open the USD file
    result, error = await omni.usd.get_context().open_stage_async(usd_path)
    
    if not result:
        print(f"[load_usd_and_play] ERROR: Failed to load USD: {error}", flush=True)
        carb.log_error(f"Failed to load USD: {error}")
        return
    
    print("[load_usd_and_play] USD file loaded successfully!", flush=True)
    
    # Wait for the stage to fully load
    print("[load_usd_and_play] Waiting for stage to stabilize...", flush=True)
    for _ in range(120):  # Wait more frames for large USD files
        await omni.kit.app.get_app().next_update_async()
    
    # Start simulation (Play)
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    
    # Wait a bit for simulation to start
    for _ in range(30):
        await omni.kit.app.get_app().next_update_async()
    
    # Create signal file to indicate ready
    with open(signal_file, 'w') as f:
        f.write('ready')
    
    print(f"[load_usd_and_play] Signal file created: {signal_file}", flush=True)
    print("Stage loaded and simulation is playing.", flush=True)
    sys.stdout.flush()

# Run the async function
asyncio.ensure_future(load_and_play())
