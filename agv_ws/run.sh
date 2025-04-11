#!/bin/bash

# Exit if any command fails
set -e

function error_exit {
    echo "Error: $1" >&2
    exit 1
}

if ! command -v colcon &> /dev/null; then
    error_exit "colcon is not installed or not in your PATH."
fi

if ! command -v ros2 &> /dev/null; then
    error_exit "ros2 is not installed or not in your PATH."
fi

echo "Building agv_ws package..."
if ! colcon build; then
    error_exit "colcon build failed."
fi

SETUP_FILE="./install/setup.bash"
if [ ! -f "$SETUP_FILE" ]; then
    error_exit "Setup file '$SETUP_FILE' does not exist."
fi

echo "Sourcing project..."
source "$SETUP_FILE"

LAUNCH_DIR="./src/control/launch"
if [ ! -d "$LAUNCH_DIR" ]; then
    error_exit "Launch directory '$LAUNCH_DIR' not found."
fi

cd "$LAUNCH_DIR"

LAUNCH_FILE="launch.py"
if [ ! -f "$LAUNCH_FILE" ]; then
    error_exit "Launch file '$LAUNCH_FILE' not found in '$LAUNCH_DIR'."
fi

echo "Launching agv_ws..."
if ! ros2 launch "$LAUNCH_FILE"; then
    error_exit "Failed to launch '$LAUNCH_FILE'."
fi
