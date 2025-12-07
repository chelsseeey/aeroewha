# Dockerfile for Drone Control System
FROM python:3.9-slim

# Install system dependencies
RUN apt-get update && apt-get install -y \
    libopencv-dev \
    python3-opencv \
    && rm -rf /var/lib/apt/lists/*

# Set working directory
WORKDIR /app

# Copy requirements and install Python dependencies
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Copy project files
COPY . .

# Create log directories
RUN mkdir -p logs/runs logs/videos

# Set environment variables
ENV PYTHONUNBUFFERED=1

# Run the application
CMD ["python3", "main.py"]
