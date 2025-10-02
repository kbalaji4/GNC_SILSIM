cmake_minimum_required(VERSION 3.10)
project(EKFSimulator)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find Eigen3
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(${EIGEN3_INCLUDE_DIR})

# Add executable
add_executable(test_ekf
    test_ekf.cpp
    ekf.cpp
)

# Link libraries
target_link_libraries(test_ekf Eigen3::Eigen)

# Compiler-specific options
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    target_compile_options(test_ekf PRIVATE -Wall -Wextra -O2)
endif()

# Set output directory
set_target_properties(test_ekf PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)
