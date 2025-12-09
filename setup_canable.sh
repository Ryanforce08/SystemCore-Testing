#!/bin/bash

# Exit on any error
set -e

# Configure CAN0 bitrate
echo "Setting CAN0 bitrate to 1Mbps..."
sudo ip link set can0 type can bitrate 1000000

# Bring CAN0 up
echo "Bringing CAN0 up..."
sudo ip link set can0 up

# Show status
echo "CAN0 status:"
ip -details -statistics link show can0

echo "CAN0 setup complete."
