#!/bin/bash

clear

# Define the remote server details
REMOTE_USER="root"                              # SSH username
REMOTE_HOST="192.168.0.14"                      # IP address of remote server
REMOTE_PATH="/root/Documents/robot/miniAGV/agv_ws/data"  # Remote path to 'data'

# Define the local workspace path
LOCAL_BASE="/home/nessy/Documents/ClasesControl"   # Base local path
LOCAL_DATA="$LOCAL_BASE/data"                     # Full local path to save 'data'

# Ensure local directory exists
if [ ! -d "$LOCAL_BASE" ]; then
    echo "Local path does not exist. Creating it..."
    mkdir -p "$LOCAL_BASE"
fi

# Transfer the 'data' folder from remote to local
echo "Starting transfer of 'data' folder from $REMOTE_HOST..."
scp -r "$REMOTE_USER@$REMOTE_HOST:$REMOTE_PATH" "$LOCAL_BASE"

# Check if SCP was successful
if [ $? -eq 0 ]; then
    echo "'data' folder successfully transferred to $LOCAL_DATA"
else
    echo "❌ Error during transfer."
    exit 1
fi

# Run the Python script to plot CSVs
echo "Executing plotCsv.py..."
python3 "$LOCAL_DATA/plotCsv.py"

# Done
echo "✅ Script complete."
