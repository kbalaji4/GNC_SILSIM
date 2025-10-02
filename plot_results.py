#!/usr/bin/env python3
"""
Python script to plot EKF simulation results
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import sys
import os

def plot_ekf_results(csv_file="ekf_results.csv"):
    """
    Plot position, velocity, and acceleration from EKF results
    """
    if not os.path.exists(csv_file):
        print(f"Error: File {csv_file} not found")
        print("Please run the C++ simulation first to generate results")
        return
    
    # Read the CSV file
    try:
        df = pd.read_csv(csv_file)
        print(f"Loaded {len(df)} data points from {csv_file}")
    except Exception as e:
        print(f"Error reading CSV file: {e}")
        return
    
    # Create subplots
    fig, axes = plt.subplots(3, 3, figsize=(15, 12))
    fig.suptitle('EKF Simulation Results - Rocket Flight Data', fontsize=16)
    
    time = df['timestamp']
    
    # Optional FSM vertical lines if present
    fsm_changes = []
    if 'fsm' in df.columns:
        prev = None
        for i, (t, s) in enumerate(zip(df['timestamp'], df['fsm'])):
            if i == 0:
                prev = s
            else:
                if s != prev:
                    fsm_changes.append((t, s))
                    prev = s
        # Also mark the first state start
        if len(df) > 0:
            fsm_changes = [(df['timestamp'].iloc[0], df['fsm'].iloc[0])] + fsm_changes
    
    # Position plots
    axes[0, 0].plot(time, df['pos_x'], 'b-', linewidth=1)
    axes[0, 0].set_title('Position X')
    axes[0, 0].set_ylabel('Position (m)')
    axes[0, 0].grid(True)
    
    axes[0, 1].plot(time, df['pos_y'], 'g-', linewidth=1)
    axes[0, 1].set_title('Position Y')
    axes[0, 1].set_ylabel('Position (m)')
    axes[0, 1].grid(True)
    
    axes[0, 2].plot(time, df['pos_z'], 'r-', linewidth=1)
    axes[0, 2].set_title('Position Z (Altitude)')
    axes[0, 2].set_ylabel('Position (m)')
    axes[0, 2].grid(True)
    
    # Velocity plots
    axes[1, 0].plot(time, df['vel_x'], 'b-', linewidth=1)
    axes[1, 0].set_title('Velocity X')
    axes[1, 0].set_ylabel('Velocity (m/s)')
    axes[1, 0].grid(True)
    
    axes[1, 1].plot(time, df['vel_y'], 'g-', linewidth=1)
    axes[1, 1].set_title('Velocity Y')
    axes[1, 1].set_ylabel('Velocity (m/s)')
    axes[1, 1].grid(True)
    
    axes[1, 2].plot(time, df['vel_z'], 'r-', linewidth=1)
    axes[1, 2].set_title('Velocity Z')
    axes[1, 2].set_ylabel('Velocity (m/s)')
    axes[1, 2].grid(True)
    
    # Acceleration plots
    axes[2, 0].plot(time, df['acc_x'], 'b-', linewidth=1)
    axes[2, 0].set_title('Acceleration X')
    axes[2, 0].set_ylabel('Acceleration (m/s²)')
    axes[2, 0].set_xlabel('Time (s)')
    axes[2, 0].grid(True)
    
    axes[2, 1].plot(time, df['acc_y'], 'g-', linewidth=1)
    axes[2, 1].set_title('Acceleration Y')
    axes[2, 1].set_ylabel('Acceleration (m/s²)')
    axes[2, 1].set_xlabel('Time (s)')
    axes[2, 1].grid(True)
    
    axes[2, 2].plot(time, df['acc_z'], 'r-', linewidth=1)
    axes[2, 2].set_title('Acceleration Z')
    axes[2, 2].set_ylabel('Acceleration (m/s²)')
    axes[2, 2].set_xlabel('Time (s)')
    axes[2, 2].grid(True)

    # Draw FSM change vertical lines across all subplots
    if fsm_changes:
        for t, label in fsm_changes:
            for ax in axes.flat:
                ax.axvline(x=t, color='k', linestyle='--', alpha=0.3)
            # place a text label at the top of the first subplot for each change
            axes[0, 0].text(t, axes[0, 0].get_ylim()[1], label, rotation=90,
                            verticalalignment='bottom', fontsize=8, color='k')
    
    plt.tight_layout()
    plt.show()
    
    # Create 3D trajectory plot
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    ax.plot(df['pos_x'], df['pos_y'], df['pos_z'], 'b-', linewidth=2, label='Rocket Trajectory')
    ax.scatter(df['pos_x'].iloc[0], df['pos_y'].iloc[0], df['pos_z'].iloc[0], 
               color='green', s=100, label='Start')
    ax.scatter(df['pos_x'].iloc[-1], df['pos_y'].iloc[-1], df['pos_z'].iloc[-1], 
               color='red', s=100, label='End')
    
    ax.set_xlabel('Position X (m)')
    ax.set_ylabel('Position Y (m)')
    ax.set_zlabel('Position Z (m)')
    ax.set_title('3D Rocket Trajectory')
    ax.legend()
    
    plt.show()
    
    # Print summary statistics
    print("\n=== EKF Simulation Summary ===")
    print(f"Total flight time: {time.iloc[-1]:.2f} seconds")
    print(f"Maximum altitude: {df['pos_z'].max():.2f} m")
    print(f"Maximum velocity magnitude: {np.sqrt(df['vel_x']**2 + df['vel_y']**2 + df['vel_z']**2).max():.2f} m/s")
    print(f"Maximum acceleration magnitude: {np.sqrt(df['acc_x']**2 + df['acc_y']**2 + df['acc_z']**2).max():.2f} m/s²")
    
    # Final position
    print(f"\nFinal position: ({df['pos_x'].iloc[-1]:.2f}, {df['pos_y'].iloc[-1]:.2f}, {df['pos_z'].iloc[-1]:.2f}) m")
    print(f"Final velocity: ({df['vel_x'].iloc[-1]:.2f}, {df['vel_y'].iloc[-1]:.2f}, {df['vel_z'].iloc[-1]:.2f}) m/s")

def main():
    if len(sys.argv) > 1:
        csv_file = sys.argv[1]
    else:
        csv_file = "ekf_results.csv"
    
    plot_ekf_results(csv_file)

if __name__ == "__main__":
    main()
