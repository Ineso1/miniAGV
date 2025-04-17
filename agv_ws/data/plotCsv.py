#!/usr/bin/env python3

import os
import glob
import pandas as pd
import matplotlib.pyplot as plt

# Path to the current directory (should be your data folder)
LOG_DIR = os.path.dirname(os.path.abspath(__file__))

def plot_velocity_logs():
    for file in glob.glob(os.path.join(LOG_DIR, 'velocity_log_*.csv')):
        df = pd.read_csv(file)
        
        # Check if the required columns exist
        if 'vx' in df.columns and 'vy' in df.columns and 'angular' in df.columns:
            plt.figure()  # Create a new figure
            df.plot(x=None, y=['vx', 'vy', 'angular'], title=f'Velocity Log - {os.path.basename(file)}')
            plt.xlabel('Sample')
            plt.ylabel('Velocity')
            plt.grid(True)
            plt.tight_layout()
        else:
            print(f"Warning: Missing velocity columns in {file}")

def plot_position_logs():
    for file in glob.glob(os.path.join(LOG_DIR, 'velocity_log_*.csv')):  # Same log file as for velocity
        df = pd.read_csv(file)
        
        # Check if the required columns for position exist
        if 'x' in df.columns and 'y' in df.columns and 'theta' in df.columns:
            plt.figure()  # Create a new figure for position plot
            df.plot(x=None, y=['x', 'y', 'theta'], title=f'Position and Orientation Log - {os.path.basename(file)}')
            plt.xlabel('Sample')
            plt.ylabel('Position / Orientation')
            plt.grid(True)
            plt.tight_layout()
        else:
            print(f"Warning: Missing position or orientation columns in {file}")

def plot_control_logs():
    for file in glob.glob(os.path.join(LOG_DIR, 'control_log_*.csv')):
        df = pd.read_csv(file)
        
        # Create a single figure for both control errors and control outputs
        plt.figure()

        # Plot errors if columns exist
        error_columns = ['error_x', 'error_y', 'error_theta']
        if all(col in df.columns for col in error_columns):
            df.plot(x=None, y=error_columns, title=f'Control Errors and Outputs - {os.path.basename(file)}', ax=plt.gca())
        else:
            print(f"Warning: Missing error columns in {file}")

        # Plot control outputs if columns exist (use v_x, v_y, v_theta instead of cmd_x, cmd_y, cmd_theta)
        control_columns = ['v_x', 'v_y', 'v_theta']
        if all(col in df.columns for col in control_columns):
            df.plot(x=None, y=control_columns, ax=plt.gca())  # Plot on the same axis
        else:
            print(f"Warning: Missing control columns in {file}")
        
        plt.xlabel('Sample')
        plt.ylabel('Values')
        plt.grid(True)
        plt.tight_layout()

def plot_trajectory_logs():
    for file in glob.glob(os.path.join(LOG_DIR, 'trajectory_log_*.csv')):
        df = pd.read_csv(file)
        
        plt.figure()
        plt.plot(df['x'], df['y'], marker='o', label='Trajectory')
        plt.title(f'Trajectory - {os.path.basename(file)}')
        plt.xlabel('X Position')
        plt.ylabel('Y Position')
        plt.axis('equal')
        plt.grid(True)
        plt.legend()
        plt.tight_layout()

def main():
    # Plot all logs: velocity, control, trajectory, and position
    plot_velocity_logs()
    plot_position_logs()  # New plot for position data
    plot_control_logs()
    plot_trajectory_logs()

    # Show the plots
    plt.show()

    # Clear the current figure to ensure no extra plots are displayed
    plt.clf()

if __name__ == '__main__':
    main()
