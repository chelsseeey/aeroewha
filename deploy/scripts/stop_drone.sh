#!/bin/bash
# Drone System Stop Script

echo "Stopping Drone Control System..."

# Find and kill the main process
pkill -f "python.*main.py"

echo "Drone system stopped."
