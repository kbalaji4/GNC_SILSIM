# EKF Flight Data Simulator

## Prerequisites

- **C++17 compatible compiler** (GCC, Clang, or MSVC)
- **Eigen3 library** for matrix operations
- **Python 3** with matplotlib, pandas, numpy for plotting

### Installing Dependencies

**macOS (using Homebrew):**
```bash
brew install eigen
pip3 install matplotlib pandas numpy
```

**Ubuntu/Debian:**
```bash
sudo apt-get install libeigen3-dev
pip3 install matplotlib pandas numpy
```

**Windows:**
- Install Eigen from: https://eigen.tuxfamily.org/
- Install Python packages: `pip install matplotlib pandas numpy`

## Building and Running

### Quick Start

1. **Build the project:**
   ```bash
   chmod +x build.sh
   ./build.sh
   ```

2. **Run with sample data:**
   ```bash
   ./test_ekf "MIDAS Sustainer (Trimmed CSV).csv" results.csv
   ```

3. **Plot the results:**
   ```bash
   python3 plot_results.py results.csv
   ```

### Using CMake (Alternative)

```bash
mkdir build
cd build
cmake ..
make
./test_ekf
```

## Usage

### Command Line Options

```bash
./test_ekf [input_csv] [output_csv]
```

- `input_csv`: Path to input CSV file (default: "MIDAS Sustainer (Trimmed CSV).csv")
- `output_csv`: Path to output CSV file (default: "ekf_results.csv")

### CSV Input Format

The program expects CSV files with the following columns:
- `timestamp`: Time in milliseconds
- `lowg.ax`, `lowg.ay`, `lowg.az`: Low-g accelerometer data
- `highg.ax`, `highg.ay`, `highg.az`: High-g accelerometer data
- `barometer.altitude`: Barometric altitude
- `orientation.yaw`, `orientation.pitch`, `orientation.roll`: Orientation data
- `fsm`: Flight state machine state

### Output Format

The program generates a CSV file with:
- `timestamp`: Time in seconds
- `pos_x`, `pos_y`, `pos_z`: Position estimates (meters)
- `vel_x`, `vel_y`, `vel_z`: Velocity estimates (m/s)
- `acc_x`, `acc_y`, `acc_z`: Acceleration estimates (m/s²)
- `altitude`: Altitude estimate (meters)

### Running the Plotter

```bash
python3 plot_results.py [results_csv]
```

### Modifying Sensor Models

Edit `sensor_data.h` to add new sensor types or modify existing structures.

### Adjusting EKF Parameters

Key parameters in `ekf.cpp`:
- `spectral_density_`: Process noise spectral density (default: 13.0)
- `s_dt`: Default time step (default: 0.05s)
- `R` matrix: Measurement noise covariance
- `Q` matrix: Process noise covariance

### Adding New Sensors

1. Add sensor structure to `sensor_data.h`
2. Update `Buffer.h` if needed
3. Modify `test_ekf.cpp` to parse new CSV columns
4. Update EKF measurement model in `ekf.cpp`

## Using run_simulation.sh

The `run_simulation.sh` script automates the entire build-run-plot workflow:

### Basic Usage

```bash
# Run with defaults (uses MIDAS Sustainer CSV, creates results.csv)
./run_simulation.sh

# Specify input and output files
./run_simulation.sh -i sample_1k.csv -o my_results.csv

# Use interactive plotting
./run_simulation.sh --interactive

# Stop simulation at specific FSM state
./run_simulation.sh -s STATE_COAST

# Skip plotting entirely
./run_simulation.sh --no-plot
```

### Command Line Options

```bash
./run_simulation.sh [options]

Options:
  -i, --input FILE        Input CSV file (default: MIDAS Sustainer (Trimmed CSV).csv)
  -o, --output FILE       Output CSV file (default: results.csv)
  -s, --stop-state STATE  Stop at FSM state (default: STATE_LANDED)
  --interactive           Use interactive plotting with zoom/pan
  --no-plot              Skip plotting step
  -h, --help             Show help message
```

### Available FSM States

- `STATE_SAFE`
- `STATE_PYRO_TEST`
- `STATE_IDLE`
- `STATE_FIRST_BOOST`
- `STATE_BURNOUT`
- `STATE_COAST`
- `STATE_APOGEE`
- `STATE_DROGUE_DEPLOY`
- `STATE_DROGUE`
- `STATE_MAIN_DEPLOY`
- `STATE_MAIN`
- `STATE_LANDED`
- `STATE_SUSTAINER_IGNITION`
- `STATE_SECOND_BOOST`
- `STATE_FIRST_SEPARATION`

### Examples

```bash
# Quick test with sample data and interactive plots
./run_simulation.sh -i sample_1k.csv --interactive

# Run until coast phase, save to specific file
./run_simulation.sh -s STATE_COAST -o coast_results.csv

# Process full dataset without plotting
./run_simulation.sh --no-plot

# Get help
./run_simulation.sh --help
```


