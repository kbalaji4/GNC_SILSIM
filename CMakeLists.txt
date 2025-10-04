cmake_minimum_required(VERSION 3.10)
project(EKFSimulatorRewritten)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})

set(SOURCES
    test_ekf_rewritten.cpp
    ekf_rewritten.cpp
)

add_executable(test_ekf_rewritten ${SOURCES})

target_link_libraries(test_ekf_rewritten Eigen3::Eigen)

target_compile_options(test_ekf_rewritten PRIVATE -Wall -Wextra -O2)

message(STATUS "Compiling with Eigen at: ${EIGEN3_INCLUDE_DIR}")
