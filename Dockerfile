FROM ubuntu:22.04

# Install system dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    libeigen3-dev \
    python3 \
    python3-pip \
    python3-dev \
    && rm -rf /var/lib/apt/lists/*

# Set working directory
WORKDIR /app

# Copy source files
COPY . .

# Install Python dependencies
RUN pip3 install -r requirements.txt

# Make scripts executable
RUN chmod +x build.sh run_simulation.sh run.sh

# Build the project
RUN ./build.sh

# Set default command
CMD ["./run_simulation.sh", "--help"]
