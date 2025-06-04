#!/bin/bash
# Script to build and launch all Sailbot ROS nodes for ROS2 Jazzy on Ubuntu 24.04

# Define colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${YELLOW}===== Starting Sailbot System =====${NC}"

# Warm-up pause
sleep 3

# Get the directory where the script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Change to workspace root
cd "$SCRIPT_DIR"
echo -e "${GREEN}Workspace directory: $(pwd)${NC}"

# Source ROS
echo -e "${YELLOW}Sourcing ROS...${NC}"
source /opt/ros/jazzy/setup.bash
sleep 2

# Check for dependencies
echo -e "${YELLOW}Checking dependencies...${NC}"
missing_deps=false

for pkg in python3-serial python3-smbus i2c-tools; do
    if ! dpkg -l | grep -q "$pkg"; then
        echo -e "${RED}Missing dependency: $pkg${NC}"
        missing_deps=true
    fi
done

if [ "$missing_deps" = true ]; then
    echo -e "${RED}Please install missing dependencies:${NC}"
    echo -e "sudo apt update && sudo apt install -y python3-serial python3-smbus i2c-tools"
    exit 1
fi

# Build the packages
echo -e "${YELLOW}Building packages...${NC}"
colcon build --packages-select path_planning sailboat_control sensors
if [ $? -ne 0 ]; then
    echo -e "${RED}Build failed! Please check errors above.${NC}"
    exit 1
fi

# Post-build delay
sleep 5

# Source the workspace
echo -e "${YELLOW}Sourcing workspace...${NC}"
source install/setup.bash
sleep 2

# Start the nodes
echo -e "${YELLOW}Starting ROS nodes...${NC}"

declare -a pids

start_node() {
    echo -e "${GREEN}Starting $1 $2...${NC}"
    ros2 run $1 $2 &
    pids+=($!)
    sleep 4  # Increased delay for stability
}

start_node sensors gps
start_node sensors rudder_control
start_node sensors winch_control
start_node sensors wind_sensor
start_node sensors radio_comm
start_node sailboat_control navigation
start_node sailboat_control state_management

echo -e "${GREEN}All nodes started!${NC}"
echo -e "${YELLOW}Running nodes (PIDs: ${pids[@]})${NC}"

trap 'echo -e "${YELLOW}Shutting down nodes...${NC}"; for pid in "${pids[@]}"; do kill "$pid" 2>/dev/null; done; echo -e "${GREEN}All nodes stopped.${NC}"; exit 0' SIGINT

echo -e "${YELLOW}Press Ctrl+C to stop all nodes${NC}"
while true; do
    sleep 1
done
