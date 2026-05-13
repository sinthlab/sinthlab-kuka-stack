#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import glob
import os

def plot_latest_trajectory():
    # Find the most recent CSV file
    current_dir = os.path.dirname(os.path.abspath(__file__))
    csv_files = glob.glob(os.path.join(current_dir, "robot_trajectory_*.csv"))
    
    if not csv_files:
        print("No trajectory CSV files found in the directory.")
        return
        
    latest_file = max(csv_files, key=os.path.getctime)
    print(f"Plotting: {latest_file}")
    
    df = pd.read_csv(latest_file)
    
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # Plot the line
    ax.plot(df['x'], df['y'], df['z'], label='End Effector Path', color='b', linewidth=2)
    
    # Mark Start and End points
    ax.scatter(df['x'].iloc[0], df['y'].iloc[0], df['z'].iloc[0], color='g', s=100, label='Start', marker='o')
    ax.scatter(df['x'].iloc[-1], df['y'].iloc[-1], df['z'].iloc[-1], color='r', s=100, label='End', marker='x')

    ax.set_xlabel('X Position (m)')
    ax.set_ylabel('Y Position (m)')
    ax.set_zlabel('Z Position (m)')
    ax.set_title('Robot Cartesian Trajectory')
    ax.legend()
    
    # Optional: ensure axis are scaled equally to avoid distortion
    max_range = np.array([df['x'].max()-df['x'].min(), 
                          df['y'].max()-df['y'].min(), 
                          df['z'].max()-df['z'].min()]).max() / 2.0
    
    mid_x = (df['x'].max()+df['x'].min()) * 0.5
    mid_y = (df['y'].max()+df['y'].min()) * 0.5
    mid_z = (df['z'].max()+df['z'].min()) * 0.5

    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)
    
    plt.show()

if __name__ == "__main__":
    import numpy as np # import inside since we need it for scaling
    plot_latest_trajectory()
