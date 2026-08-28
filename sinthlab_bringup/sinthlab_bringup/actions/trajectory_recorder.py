import os
import csv
import time
import numpy as np

class TrajectoryRecorder:
    def __init__(self, save_dir="~/lbr-stack/src/sinthlab-kuka-stack/analysis"):
        self.save_dir = os.path.expanduser(save_dir)
        self._data = []
        self._start_time = None
        self._active = False

    def start(self, extra_header=None):
        """extra_header: optional list of column names appended after time,x,y,z. The caller then
        passes a matching list to record_pose(). Used by the maze to log MAZE-RELATIVE coordinates
        alongside the absolute pose, so a run can be plotted straight against the corridor rectangles."""
        self._data = []
        self._extra_header = list(extra_header) if extra_header else []
        self._start_time = time.time()
        self._active = True

    def record_pose(self, transform: np.ndarray, extra=None):
        if not self._active or self._start_time is None:
            return
        
        try:
            current_time = time.time() - self._start_time
            x = float(transform[0, 3])
            y = float(transform[1, 3])
            z = float(transform[2, 3])
            row = [current_time, x, y, z]
            if extra:
                row.extend(extra)
            self._data.append(row)
        except Exception:
            pass # Fail silently so we don't crash real-time loop

    def stop_and_save(self, logger=None):
        self._active = False
        if not self._data:
            return

        try:
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            filename = f"robot_trajectory_{timestamp}.csv"
            os.makedirs(self.save_dir, exist_ok=True)
            filepath = os.path.join(self.save_dir, filename)
            
            with open(filepath, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(["time", "x", "y", "z"] + getattr(self, "_extra_header", []))
                writer.writerows(self._data)
            
            if logger:
                logger.info(f"Saved {len(self._data)} trajectory points to {filepath}")
            else:
                print(f"Saved {len(self._data)} trajectory points to {filepath}")
                
        except Exception as e:
            if logger:
                logger.error(f"Failed to save trajectory: {e}")
            else:
                print(f"Failed to save trajectory: {e}")
        finally:
            # Wipe memory out
            self._data = []
