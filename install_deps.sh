#!/bin/bash

# Cross-platform dependency installer for EKF Simulator

set -e

echo "Installing dependencies for EKF Simulator..."

# Detect OS
if [[ "$OSTYPE" == "linux-gnu"* ]]; then
    OS="linux"
elif [[ "$OSTYPE" == "darwin"* ]]; then
    OS="macos"
elif [[ "$OSTYPE" == "msys" ]] || [[ "$OSTYPE" == "cygwin" ]]; then
    OS="windows"
else
    echo "Unknown OS: $OSTYPE"
    exit 1
fi

echo "Detected OS: $OS"

# Install system dependencies
if [[ "$OS" == "linux" ]]; then
    echo "Installing Linux dependencies..."
    sudo apt-get update
    sudo apt-get install -y build-essential cmake libeigen3-dev python3 python3-pip python3-dev
    
elif [[ "$OS" == "macos" ]]; then
    echo "Installing macOS dependencies..."
    if ! command -v brew &> /dev/null; then
        echo "Homebrew not found. Please install Homebrew first:"
        echo "https://brew.sh/"
        exit 1
    fi
    brew install eigen cmake python3
    
elif [[ "$OS" == "windows" ]]; then
    echo "Installing Windows dependencies..."
    echo "Please install the following manually:"
    echo "1. Visual Studio Build Tools or MinGW-w64"
    echo "2. CMake: https://cmake.org/download/"
    echo "3. Eigen: https://eigen.tuxfamily.org/"
    echo "4. Python 3: https://python.org/downloads/"
    echo ""
    echo "Or use Windows Subsystem for Linux (WSL) for easier setup."
fi

# Install Python dependencies
echo "Installing Python dependencies..."
pip3 install -r requirements.txt

echo "Dependencies installed successfully!"
echo ""
echo "To run the simulation:"
echo "  ./run_simulation.sh"
echo ""
echo "For Docker users:"
echo "  docker build -t ekf-simulator ."
echo "  docker run -v \$(pwd)/data:/app/data -v \$(pwd)/output:/app/output ekf-simulator"
