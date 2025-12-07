#!/bin/bash
# Setup Script for Drone Project

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$PROJECT_ROOT"

echo "Setting up Drone Control System..."

# Create virtual environment
if [ ! -d "venv" ]; then
    echo "Creating virtual environment..."
    python3 -m venv venv
fi

# Activate virtual environment
source venv/bin/activate

# Upgrade pip
echo "Upgrading pip..."
pip install --upgrade pip

# Install dependencies
echo "Installing dependencies..."
pip install -r requirements.txt

# Create log directories
echo "Creating log directories..."
mkdir -p logs/runs
mkdir -p logs/videos

echo "Setup complete!"
echo "To activate the environment: source venv/bin/activate"
echo "To start the drone system: ./deploy/scripts/start_drone.sh"
