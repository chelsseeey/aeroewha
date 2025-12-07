#!/bin/bash
# Drone System Startup Script

# Set project root
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$PROJECT_ROOT"

echo "Starting Drone Control System..."
echo "Project root: $PROJECT_ROOT"

# Activate virtual environment if it exists
if [ -d "venv" ]; then
    echo "Activating virtual environment..."
    source venv/bin/activate
fi

# Check if config files exist
if [ ! -f "configs/system.yaml" ]; then
    echo "Error: Configuration files not found!"
    exit 1
fi

# Start the main application
echo "Launching main.py..."
python3 main.py "$@"
