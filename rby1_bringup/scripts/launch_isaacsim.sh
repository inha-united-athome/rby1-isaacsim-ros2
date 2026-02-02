#!/bin/bash
#
# Isaac Sim Launch Script for RBy1 Robot
# Usage: ./launch_isaacsim.sh [usd_file_path]
#

# Default USD file path (change this to your USD file)
DEFAULT_USD_FILE="${HOME}/isaac_custom_assets/rby1_0813/model_v1.1/model_v1.1.usd"

# Isaac Sim installation path
ISAAC_SIM_PATH="${HOME}/.local/share/ov/pkg/isaac-sim-4.2.0"

# Parse arguments
USD_FILE="${1:-$DEFAULT_USD_FILE}"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}======================================${NC}"
echo -e "${GREEN}  Isaac Sim Launcher for RBy1 Robot  ${NC}"
echo -e "${GREEN}======================================${NC}"

# Check if Isaac Sim exists
if [ ! -d "$ISAAC_SIM_PATH" ]; then
    echo -e "${YELLOW}Isaac Sim not found at: $ISAAC_SIM_PATH${NC}"
    echo -e "${YELLOW}Searching for Isaac Sim installation...${NC}"
    
    # Try to find Isaac Sim
    ISAAC_SIM_PATH=$(find "${HOME}/.local/share/ov/pkg" -maxdepth 1 -name "isaac-sim*" -type d 2>/dev/null | head -1)
    
    if [ -z "$ISAAC_SIM_PATH" ]; then
        echo -e "${RED}Error: Isaac Sim installation not found!${NC}"
        echo -e "${RED}Please install Isaac Sim or set ISAAC_SIM_PATH manually.${NC}"
        exit 1
    fi
    echo -e "${GREEN}Found Isaac Sim at: $ISAAC_SIM_PATH${NC}"
fi

# Check if USD file exists
if [ ! -f "$USD_FILE" ]; then
    echo -e "${RED}Error: USD file not found: $USD_FILE${NC}"
    echo -e "${YELLOW}Available USD files:${NC}"
    find "${HOME}/isaac_custom_assets" -name "*.usd" 2>/dev/null | head -10
    echo ""
    echo -e "${YELLOW}Usage: $0 [path_to_usd_file]${NC}"
    exit 1
fi

echo -e "${GREEN}USD File: $USD_FILE${NC}"
echo -e "${GREEN}Isaac Sim: $ISAAC_SIM_PATH${NC}"
echo ""

# Source ROS 2 environment
echo -e "${YELLOW}Sourcing ROS 2 environment...${NC}"
source /opt/ros/humble/setup.bash

# Source workspace if exists
WORKSPACE_SETUP="${HOME}/rby1_ros2_ws/install/setup.bash"
if [ -f "$WORKSPACE_SETUP" ]; then
    source "$WORKSPACE_SETUP"
    echo -e "${GREEN}Sourced workspace: $WORKSPACE_SETUP${NC}"
fi

# Launch Isaac Sim with the USD file
echo -e "${GREEN}Launching Isaac Sim...${NC}"
echo ""

cd "$ISAAC_SIM_PATH"

# Option 1: Launch with USD file directly
./isaac-sim.sh --open "$USD_FILE"

# Option 2: If you want to run in headless mode, use:
# ./isaac-sim.sh --headless --open "$USD_FILE"

# Option 3: If you want to run a Python script:
# ./python.sh /path/to/your/script.py
