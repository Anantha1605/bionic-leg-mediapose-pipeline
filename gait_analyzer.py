import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import find_peaks

# ---------------- Load CSV ----------------
csv_file = "output_angles.csv"  
df = pd.read_csv(csv_file)

# ---------------- Extract & Correct Joint Angle Series ----------------
# Corrected so that 0° = fully extended leg
right_knee = 180 - df['right_knee_angle'].values
left_knee  = 180 - df['left_knee_angle'].values
right_hip  = 180 - df['right_hip_angle'].values
left_hip   = 180 - df['left_hip_angle'].values
frames     = df['frame'].values

# ---------------- Detect Gait Cycles ----------------
# Knee minima ≈ heel strike events
# distance=20 ensures minimum separation between peaks (adjust for your frame rate)
right_peaks, _ = find_peaks(-right_knee, distance=20)
left_peaks, _  = find_peaks(-left_knee, distance=20)

print(f"Detected {len(right_peaks)} right leg steps, {len(left_peaks)} left leg steps")

# ---------------- Compute Step Durations ----------------
right_step_durations = np.diff(right_peaks)  # in frames
left_step_durations  = np.diff(left_peaks)

print(f"Average right step duration: {np.mean(right_step_durations):.1f} frames")
print(f"Average left step duration: {np.mean(left_step_durations):.1f} frames")

# ---------------- Compute Range of Motion ----------------
def range_of_motion(series, peaks):
    rom = []
    for i in range(len(peaks)-1):
        segment = series[peaks[i]:peaks[i+1]]
        rom.append(np.max(segment) - np.min(segment))
    return rom

right_knee_rom = range_of_motion(right_knee, right_peaks)
left_knee_rom  = range_of_motion(left_knee, left_peaks)
right_hip_rom  = range_of_motion(right_hip, right_peaks)
left_hip_rom   = range_of_motion(left_hip, left_peaks)

print(f"Average right knee ROM: {np.mean(right_knee_rom):.1f}°")
print(f"Average left knee ROM: {np.mean(left_knee_rom):.1f}°")
print(f"Average right hip ROM: {np.mean(right_hip_rom):.1f}°")
print(f"Average left hip ROM: {np.mean(left_hip_rom):.1f}°")

# ---------------- Plot Results ----------------
plt.figure(figsize=(12, 8))

# Right leg
plt.subplot(2,1,1)
plt.plot(frames, right_knee, label='Right Knee', color='blue')
plt.plot(frames, right_hip, label='Right Hip', color='green')
# Draw vertical dotted lines at heel strikes
for peak in right_peaks:
    plt.axvline(x=frames[peak], color='red', linestyle='--', alpha=0.7)
plt.title('Right Leg Joint Angles (0° = fully extended)')
plt.ylabel('Angle (deg)')
plt.legend()
plt.grid(True)

# Left leg
plt.subplot(2,1,2)
plt.plot(frames, left_knee, label='Left Knee', color='blue')
plt.plot(frames, left_hip, label='Left Hip', color='green')
# Draw vertical dotted lines at heel strikes
for peak in left_peaks:
    plt.axvline(x=frames[peak], color='red', linestyle='--', alpha=0.7)
plt.title('Left Leg Joint Angles (0° = fully extended)')
plt.xlabel('Frame')
plt.ylabel('Angle (deg)')
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.show()
