#!/bin/bash

# === Color Definitions ===
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[1;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# === Spinner for animations ===
function spinner() {
    local pid=$!
    local delay=0.15
    local spinstr='|/-\'
    while [ "$(ps a | awk '{print $1}' | grep $pid)" ]; do
        local temp=${spinstr#?}
        printf " [%c]  " "$spinstr"
        local spinstr=$temp${spinstr%"$temp"}
        sleep $delay
        printf "\b\b\b\b\b\b"
    done
    wait $pid
    return $?
}

function error_exit {
    echo -e "${RED}❌ Error: $1${NC}" >&2
    exit 1
}

echo -e "${CYAN}🚦 ROS2 Control Node Launcher${NC}"
echo -e "${CYAN}------------------------------${NC}"

# === Step 1: Clean old CSVs ===
DATA_DIR="./data"
echo -ne "${YELLOW}🧹 Cleaning old CSV files...${NC}"
if [ -d "$DATA_DIR" ]; then
    rm -f "$DATA_DIR"/*.csv & spinner
    echo -e "${GREEN} Done!${NC}"
else
    echo -e "${RED} Directory not found! Skipping.${NC}"
fi

# === Step 2: Check required commands ===
echo -ne "${YELLOW}🔍 Checking ROS2...${NC}"
if ! command -v ros2 &>/dev/null; then
    error_exit "ros2 is not installed or not in PATH."
fi
echo -e "${GREEN} Found!${NC}"

# === Step 3: Source workspace ===
SETUP_FILE="./install/setup.bash"
if [ ! -f "$SETUP_FILE" ]; then
    error_exit "Setup file '$SETUP_FILE' not found. Did you build the workspace?"
fi
echo -e "${YELLOW}📡 Sourcing workspace...${NC}"
source "$SETUP_FILE"

# === Step 4: Launch ===
LAUNCH_DIR="./src/control/launch"
LAUNCH_FILE="launch.py"

if [ ! -d "$LAUNCH_DIR" ]; then
    error_exit "Launch directory '$LAUNCH_DIR' not found."
fi

if [ ! -f "$LAUNCH_DIR/$LAUNCH_FILE" ]; then
    error_exit "Launch file '$LAUNCH_FILE' not found in $LAUNCH_DIR."
fi

echo -e "${CYAN}🚀 Launching control node...${NC}"
ros2 launch control "$LAUNCH_FILE"
