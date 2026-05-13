import time
import numpy as np
import os
import csv

class TrajectoryRecorder:
    def __init__(self, filename="trajectory.csv"):
        self.filename = filename
        self.clear()

    def record(self, time_sec, x, y, z):
        self.data.append([time_sec, x, y, z])

    def clear(self):
        self.data = []

    def save(self):
        if not self.data:
            return
        
        # Determine unique filename if saving multiple times
        base, ext = os.path.splitext(self.filename)
        out_file = self.filename
        counter = 1
        while os.path.exists(out_file):
            out_file = f"{base}_{counter}{ext}"
            counter += 1
            
        with open(out_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(["time", "x", "y", "z"])
            writer.writerows(self.data)
        print(f"Saved trajectory to {out_file}")

recorder = TrajectoryRecorder(os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../analysis/robot_trajectory.csv")))
