#!/bin/bash

# EKF Simulation Runner Script
# This script builds, runs the simulation, and plots results in one command

set -e  # Exit on any error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Default values
INPUT_FILE="MIDAS Sustainer (Trimmed CSV).csv"
OUTPUT_FILE="results.csv"
PLOT_RESULTS=true
STOP_STATE="STATE_LANDED"

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -i|--input)
            INPUT_FILE="$2"
            shift 2
            ;;
        -o|--output)
            OUTPUT_FILE="$2"
            shift 2
            ;;
        --no-plot)
            PLOT_RESULTS=false
            shift
            ;;
        -s|--stop-state)
            STOP_STATE="$2"
            shift 2
            ;;
        -h|--help)
            echo "Usage: $0 [OPTIONS]"
            echo "Options:"
            echo "  -i, --input FILE     Input CSV file (default: sample_1k.csv)"
            echo "  -o, --output FILE    Output CSV file (default: results.csv)"
            echo "  -s, --stop-state     FSM state to stop simulation at (default: STATE_LANDED)"
            echo "  --no-plot           Skip plotting results"
            echo "  -h, --help          Show this help message"
            echo ""
            echo "Examples:"
            echo "  $0                                    # Use defaults"
            echo "  $0 -i sample_1k.csv -o results.csv   # Specify files"
            echo "  $0 -s STATE_COAST                    # Stop at coast phase"
            echo "  $0 --no-plot                         # Skip plotting"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Use -h or --help for usage information"
            exit 1
            ;;
    esac
done

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}    EKF Flight Data Simulator Runner    ${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# Check if input file exists
if [ ! -f "$INPUT_FILE" ]; then
    echo -e "${RED}Error: Input file '$INPUT_FILE' not found!${NC}"
    echo "Available CSV files:"
    ls -la *.csv 2>/dev/null || echo "No CSV files found in current directory"
    exit 1
fi

echo -e "${YELLOW}Configuration:${NC}"
echo "  Input file:  $INPUT_FILE"
echo "  Output file: $OUTPUT_FILE"
echo "  Stop state:  $STOP_STATE"
echo "  Plot results: $PLOT_RESULTS"
echo ""

# Step 1: Build the project
echo -e "${YELLOW}Step 1: Building project...${NC}"
if [ -f "build.sh" ]; then
    ./build.sh
    if [ $? -ne 0 ]; then
        echo -e "${RED}Build failed!${NC}"
        exit 1
    fi
    echo -e "${GREEN}Build completed successfully!${NC}"
else
    echo -e "${RED}Error: build.sh not found!${NC}"
    exit 1
fi
echo ""

# Step 2: Run the simulation
echo -e "${YELLOW}Step 2: Running EKF simulation...${NC}"
if [ -f "./test_ekf" ]; then
    ./test_ekf "$INPUT_FILE" "$OUTPUT_FILE" "$STOP_STATE"
    if [ $? -ne 0 ]; then
        echo -e "${RED}Simulation failed!${NC}"
        exit 1
    fi
    echo -e "${GREEN}Simulation completed successfully!${NC}"
    echo "Results saved to: $OUTPUT_FILE"
else
    echo -e "${RED}Error: test_ekf executable not found!${NC}"
    echo "Make sure the build completed successfully."
    exit 1
fi
echo ""

# Step 3: Plot results (if requested)
if [ "$PLOT_RESULTS" = true ]; then
    echo -e "${YELLOW}Step 3: Plotting results...${NC}"
    if [ -f "plot_results.py" ]; then
        python3 plot_results.py "$OUTPUT_FILE"
        if [ $? -ne 0 ]; then
            echo -e "${RED}Plotting failed!${NC}"
            echo "You can still view the results in: $OUTPUT_FILE"
        else
            echo -e "${GREEN}Plots generated successfully!${NC}"
        fi
    else
        echo -e "${RED}Error: plot_results.py not found!${NC}"
        echo "Results are available in: $OUTPUT_FILE"
    fi
else
    echo -e "${YELLOW}Step 3: Skipping plots (--no-plot specified)${NC}"
fi
echo ""

# Step 4: Show summary
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}           SIMULATION COMPLETE           ${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo -e "${BLUE}Summary:${NC}"
echo "  ✓ Project built successfully"
echo "  ✓ Simulation completed"
echo "  ✓ Results saved to: $OUTPUT_FILE"

if [ "$PLOT_RESULTS" = true ]; then
    echo "  ✓ Plots generated"
fi

echo ""
echo -e "${BLUE}Next steps:${NC}"
echo "  • View results: cat $OUTPUT_FILE"
echo "  • Plot results: python3 plot_results.py $OUTPUT_FILE"
echo "  • Run with different input: $0 -i your_file.csv -o your_results.csv"
echo ""
echo -e "${GREEN}Done! 🚀${NC}"
