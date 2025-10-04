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
    
    # Create first figure with standalone KF plots (3x3 layout)
    fig1, axes1 = plt.subplots(3, 3, figsize=(15, 12))
    fig1.suptitle('EKF Simulation Results - Standalone Plots', fontsize=16)
    
    # Create second figure with comparison plots (2x2 layout)
    fig2, axes2 = plt.subplots(2, 2, figsize=(12, 10))
    fig2.suptitle('EKF Simulation Results - Comparison Plots', fontsize=16)
    
    time = df['timestamp']
    
    # Optional FSM vertical lines if present
    fsm_changes = []
    used_states_order = []
    if 'fsm' in df.columns:
        prev = None
        for i, (t, s) in enumerate(zip(df['timestamp'], df['fsm'])):
            if i == 0:
                prev = s
                fsm_changes.append((t, s))
                if s not in used_states_order:
                    used_states_order.append(s)
            else:
                if s != prev:
                    fsm_changes.append((t, s))
                    if s not in used_states_order:
                        used_states_order.append(s)
                    prev = s
    
    # Position plots (Figure 1 - Standalone)
    axes1[0, 0].plot(time, df['pos_x'], 'b-', linewidth=1)
    axes1[0, 0].set_title('Position X')
    axes1[0, 0].set_ylabel('Position (m)')
    axes1[0, 0].grid(True)
    
    axes1[0, 1].plot(time, df['pos_y'], 'g-', linewidth=1)
    axes1[0, 1].set_title('Position Y')
    axes1[0, 1].set_ylabel('Position (m)')
    axes1[0, 1].grid(True)
    
    axes1[0, 2].plot(time, df['pos_z'], 'r-', linewidth=1)
    axes1[0, 2].set_title('Position Z (Altitude)')
    axes1[0, 2].set_ylabel('Position (m)')
    axes1[0, 2].grid(True)
    
    # Velocity plots (Figure 1 - Standalone)
    axes1[1, 0].plot(time, df['vel_x'], 'b-', linewidth=1)
    axes1[1, 0].set_title('Velocity X')
    axes1[1, 0].set_ylabel('Velocity (m/s)')
    axes1[1, 0].grid(True)
    
    axes1[1, 1].plot(time, df['vel_y'], 'g-', linewidth=1)
    axes1[1, 1].set_title('Velocity Y')
    axes1[1, 1].set_ylabel('Velocity (m/s)')
    axes1[1, 1].grid(True)
    
    axes1[1, 2].plot(time, df['vel_z'], 'r-', linewidth=1)
    axes1[1, 2].set_title('Velocity Z')
    axes1[1, 2].set_ylabel('Velocity (m/s)')
    axes1[1, 2].grid(True)
    
    # Acceleration plots (Figure 1 - Standalone)
    axes1[2, 0].plot(time, df['acc_x'], 'b-', linewidth=1)
    axes1[2, 0].set_title('Acceleration X')
    axes1[2, 0].set_ylabel('Acceleration (m/s²)')
    axes1[2, 0].set_xlabel('Time (s)')
    axes1[2, 0].grid(True)
    
    axes1[2, 1].plot(time, df['acc_y'], 'g-', linewidth=1)
    axes1[2, 1].set_title('Acceleration Y')
    axes1[2, 1].set_ylabel('Acceleration (m/s²)')
    axes1[2, 1].set_xlabel('Time (s)')
    axes1[2, 1].grid(True)
    
    axes1[2, 2].plot(time, df['acc_z'], 'r-', linewidth=1)
    axes1[2, 2].set_title('Acceleration Z')
    axes1[2, 2].set_ylabel('Acceleration (m/s²)')
    axes1[2, 2].set_xlabel('Time (s)')
    axes1[2, 2].grid(True)
    
    # Comparison plots (Figure 2)
    if 'raw_highg_ax' in df.columns and 'raw_baro_alt' in df.columns:
        # Convert raw highG data from g to m/s² for comparison
        raw_acc_x = df['raw_highg_ax'] * 9.81
        raw_acc_y = df['raw_highg_ay'] * 9.81  
        raw_acc_z = df['raw_highg_az'] * 9.81
        
        # KF pos Z vs baro altitude
        axes2[0, 0].plot(time, df['pos_z'], 'b-', linewidth=2, label='KF Z Position')
        axes2[0, 0].plot(time, df['raw_baro_alt'], 'r--', linewidth=1, alpha=0.7, label='Raw Barometer Altitude')
        axes2[0, 0].set_title('KF Position Z vs Barometric Altitude')
        axes2[0, 0].set_ylabel('Altitude (m)')
        axes2[0, 0].set_xlabel('Time (s)')
        axes2[0, 0].legend()
        axes2[0, 0].grid(True)
        
        # KF accel Z vs high accel X
        axes2[0, 1].plot(time, df['acc_z'], 'b-', linewidth=2, label='KF Z Acceleration')
        axes2[0, 1].plot(time, raw_acc_x, 'r--', linewidth=1, alpha=0.7, label='Raw HighG X')
        axes2[0, 1].set_title('KF Acceleration Z vs Raw HighG X')
        axes2[0, 1].set_ylabel('Acceleration (m/s²)')
        axes2[0, 1].set_xlabel('Time (s)')
        axes2[0, 1].legend()
        axes2[0, 1].grid(True)
        
        # KF accel Y vs high accel Y
        axes2[1, 0].plot(time, df['acc_y'], 'g-', linewidth=2, label='KF Y Acceleration')
        axes2[1, 0].plot(time, raw_acc_y, 'r--', linewidth=1, alpha=0.7, label='Raw HighG Y')
        axes2[1, 0].set_title('KF Acceleration Y vs Raw HighG Y')
        axes2[1, 0].set_ylabel('Acceleration (m/s²)')
        axes2[1, 0].set_xlabel('Time (s)')
        axes2[1, 0].legend()
        axes2[1, 0].grid(True)
        
        # KF accel X vs high accel Z
        axes2[1, 1].plot(time, df['acc_x'], 'b-', linewidth=2, label='KF X Acceleration')
        axes2[1, 1].plot(time, raw_acc_z, 'r--', linewidth=1, alpha=0.7, label='Raw HighG Z')
        axes2[1, 1].set_title('KF Acceleration X vs Raw HighG Z')
        axes2[1, 1].set_ylabel('Acceleration (m/s²)')
        axes2[1, 1].set_xlabel('Time (s)')
        axes2[1, 1].legend()
        axes2[1, 1].grid(True)
    else:
        # Fallback if no raw data
        axes2[0, 0].text(0.5, 0.5, 'Raw sensor data not available', ha='center', va='center', transform=axes2[0, 0].transAxes)
        axes2[0, 0].set_title('KF Position Z vs Barometric Altitude')
        axes2[0, 1].text(0.5, 0.5, 'Raw sensor data not available', ha='center', va='center', transform=axes2[0, 1].transAxes)
        axes2[0, 1].set_title('KF Acceleration Z vs Raw HighG X')
        axes2[1, 0].text(0.5, 0.5, 'Raw sensor data not available', ha='center', va='center', transform=axes2[1, 0].transAxes)
        axes2[1, 0].set_title('KF Acceleration Y vs Raw HighG Y')
        axes2[1, 1].text(0.5, 0.5, 'Raw sensor data not available', ha='center', va='center', transform=axes2[1, 1].transAxes)
        axes2[1, 1].set_title('KF Acceleration X vs Raw HighG Z')
    

    # Draw FSM change vertical lines across all subplots
    if fsm_changes:
        # draw change lines for both figures
        for t, label in fsm_changes:
            for ax in axes1.flat:
                ax.axvline(x=t, color='k', linestyle='--', alpha=0.25)
            for ax in axes2.flat:
                ax.axvline(x=t, color='k', linestyle='--', alpha=0.25)

        # annotate only the first occurrence per unique state to avoid clutter
        annotated = set()
        ylim_top1 = axes1[0, 0].get_ylim()[1]
        ylim_top2 = axes2[0, 0].get_ylim()[1]
        for t, label in fsm_changes:
            if label not in annotated:
                axes1[0, 0].text(t, ylim_top1, label, rotation=90,
                                verticalalignment='bottom', fontsize=8, color='k')
                axes2[0, 0].text(t, ylim_top2, label, rotation=90,
                                verticalalignment='bottom', fontsize=8, color='k')
                annotated.add(label)

        # add a compact legend listing only used FSM states
        if used_states_order:
            legend_text = 'FSM states used: ' + ', '.join(used_states_order)
            fig1.text(0.5, 0.98, legend_text, ha='center', va='top', fontsize=9)
            fig2.text(0.5, 0.98, legend_text, ha='center', va='top', fontsize=9)
    
    # Show both figures
    fig1.tight_layout()
    fig1.show()
    
    fig2.tight_layout()
    fig2.show()
    
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
