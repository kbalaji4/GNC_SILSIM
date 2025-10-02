# EKF Flight Data Simulator

This project provides a standalone test harness for your Extended Kalman Filter (EKF) designed for rocket flight data analysis. It reads flight data from a CSV file and simulates the EKF algorithm to estimate position, velocity, and acceleration.

## Features

- **CSV Data Processing**: Reads flight data from CSV files with sensor readings
- **EKF Simulation**: Runs your EKF algorithm on historical flight data
- **Real-time Visualization**: Plots position, velocity, and acceleration in 3D
- **Modular Design**: Easy to modify and extend for different flight scenarios

## Files Structure

```
├── ekf.h                    # EKF class definition
├── ekf.cpp                  # EKF implementation
├── kalman_filter.h          # Base Kalman filter class
├── sensor_data.h            # Data structures for sensors
├── systems.h               # Rocket systems definitions
├── Buffer.h                # Circular buffer implementation
├── definitions.h            # Mock definitions for standalone testing
├── test_ekf.cpp            # Main test program
├── plot_results.py         # Python plotting script
├── build.sh                # Build script
├── CMakeLists.txt          # CMake configuration
└── README.md               # This file
```

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

## Visualization

The Python plotting script (`plot_results.py`) generates:

1. **3x3 Grid Plot**: Position, velocity, and acceleration for X, Y, Z axes
2. **3D Trajectory**: 3D visualization of the rocket's flight path
3. **Summary Statistics**: Flight time, max altitude, max velocity, etc.

### Running the Plotter

```bash
python3 plot_results.py [results_csv]
```

## Algorithm Details

The EKF implementation includes:

- **9-State Model**: Position, velocity, and acceleration in 3D
- **Aerodynamic Modeling**: Drag coefficients based on Mach number
- **Thrust Modeling**: Motor thrust curves for booster and sustainer stages
- **Sensor Fusion**: Barometer, accelerometer, and orientation data
- **State Machine**: Flight phase detection and handling

### Key Features

- **Adaptive Process Noise**: Q matrix updates based on time step
- **Sensor Bias Correction**: Accelerometer bias compensation
- **Coordinate Transformations**: Body-to-global frame conversions
- **Thrust Vectoring**: Motor thrust direction modeling

## Customization

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

## Troubleshooting

### Common Issues

1. **Eigen not found**: Install Eigen3 library
2. **Python plotting fails**: Install required Python packages
3. **CSV parsing errors**: Check CSV format matches expected columns
4. **Memory issues**: Reduce data size or increase system memory

### Debug Mode

Compile with debug flags:
```bash
g++ -std=c++17 -g -O0 -Wall -Wextra -I"$EIGEN_INCLUDE" -I. test_ekf.cpp ekf.cpp -o test_ekf_debug
```

## Performance

- **Processing Speed**: ~1000 data points per second
- **Memory Usage**: ~50MB for 1M data points
- **Accuracy**: Depends on sensor quality and EKF tuning

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test thoroughly
5. Submit a pull request

## License

This project is part of the GNC_SILSIM rocket simulation system.

## Support

For issues and questions:
1. Check the troubleshooting section
2. Review the code comments
3. Create an issue with detailed error messages
