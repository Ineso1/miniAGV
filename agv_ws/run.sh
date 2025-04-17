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

echo -e "${CYAN}🚧 ROS2 Workspace Build & Launch Script${NC}"
echo -e "${CYAN}---------------------------------------${NC}"

# === Step 1: Clean data ===
DATA_DIR="./data"
echo -ne "${YELLOW}🧹 Cleaning old CSV files...${NC}"
if [ -d "$DATA_DIR" ]; then
    rm -f "$DATA_DIR"/*.csv & spinner
    echo -e "${GREEN} Done!${NC}"
else
    echo -e "${RED} Directory not found! Skipping.${NC}"
fi

# === Step 2: Check dependencies ===
echo -ne "${YELLOW}🔍 Checking dependencies...${NC}"
for cmd in colcon ros2; do
    if ! command -v $cmd &>/dev/null; then
        error_exit "$cmd is not installed or not in PATH."
    fi
done
echo -e "${GREEN} All good!${NC}"

# === Step 3: Build ===
echo -ne "${BLUE}🔧 Building workspace with colcon...${NC}"
colcon build --parallel-workers 12 & spinner
echo -e "${GREEN} Build completed!${NC}"

# === Step 4: Source setup ===
SETUP_FILE="./install/setup.bash"
if [ ! -f "$SETUP_FILE" ]; then
    error_exit "Setup file '$SETUP_FILE' not found."
fi
echo -e "${YELLOW}📡 Sourcing setup.bash...${NC}"
source "$SETUP_FILE"

# === Step 5: Launch project ===
LAUNCH_DIR="./src/control/launch"
LAUNCH_FILE="launch.py"

if [ ! -d "$LAUNCH_DIR" ]; then
    error_exit "Launch directory '$LAUNCH_DIR' not found."
fi

cd "$LAUNCH_DIR"

if [ ! -f "$LAUNCH_FILE" ]; then
    error_exit "Launch file '$LAUNCH_FILE' not found."
fi

echo -e "${CYAN}🚀 Launching project: ${LAUNCH_FILE}...${NC}"
ros2 launch "$LAUNCH_FILE"
