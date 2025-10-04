cmake_minimum_required(VERSION 3.10)
project(EKFSimulatorRewritten)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find Eigen
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(${EIGEN3_INCLUDE_DIR})

# Source files
set(SOURCES
    test_ekf_rewritten.cpp
    ekf_rewritten.cpp
)

# Create executable
add_executable(test_ekf_rewritten ${SOURCES})

# Link Eigen
target_link_libraries(test_ekf_rewritten Eigen3::Eigen)

# Compiler flags
target_compile_options(test_ekf_rewritten PRIVATE -Wall -Wextra -O2)

# Print Eigen location
message(STATUS "Compiling with Eigen at: ${EIGEN3_INCLUDE_DIR}")
