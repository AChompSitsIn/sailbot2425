#!/bin/bash
# Script to build and launch all Sailbot ROS nodes
# Place this in your sailbot_ws directory and make it executable:
# chmod +x start_sailbot.sh

# Define colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${YELLOW}===== Starting Sailbot System =====${NC}"

# Get the directory where the script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Change to workspace root
cd "$SCRIPT_DIR"
echo -e "${GREEN}Workspace directory: $(pwd)${NC}"

# Source ROS
echo -e "${YELLOW}Sourcing ROS...${NC}"
source /opt/ros/humble/setup.bash

# Build the packages
echo -e "${YELLOW}Building packages...${NC}"
colcon build --packages-select path_planning sailboat_control sensors
if [ $? -ne 0 ]; then
    echo -e "${RED}Build failed! Please check errors above.${NC}"
    exit 1
fi

# Source the workspace
echo -e "${YELLOW}Sourcing workspace...${NC}"
source install/setup.bash

# Start the nodes
echo -e "${YELLOW}Starting ROS nodes...${NC}"

# Array to store process IDs
declare -a pids

# Function to start a node
start_node() {
    echo -e "${GREEN}Starting $1 $2...${NC}"
    ros2 run $1 $2 &
    pids+=($!)
    sleep 2  # Give the node time to initialize
}

# Launch the nodes in sequence
start_node sensors gps
start_node sensors rudder_control
start_node sensors winch_control
start_node sensors wind_sensor
start_node sensors radio_comm
start_node sailboat_control navigation
start_node sailboat_control state_management

echo -e "${GREEN}All nodes started!${NC}"
echo -e "${YELLOW}Running nodes (PIDs: ${pids[@]})${NC}"

# Handle clean shutdown on Ctrl+C
trap 'echo -e "${YELLOW}Shutting down nodes...${NC}"; for pid in "${pids[@]}"; do kill "$pid" 2>/dev/null; done; echo -e "${GREEN}All nodes stopped.${NC}"; exit 0' SIGINT

# Keep the script running to maintain the processes
echo -e "${YELLOW}Press Ctrl+C to stop all nodes${NC}"
while true; do
    sleep 1
done