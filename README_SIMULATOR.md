# 🎮 Robot Leg Simulator - Visual Gait Analysis

Real-time visual simulation of bionic leg movement using joint angles from MediaPose CSV data.

![Simulator Demo](https://via.placeholder.com/800x400/1a1a1e/ffffff?text=Bionic+Leg+Simulator)

## 🎯 Features

- ✅ **Real-time 2D visualization** of both legs (6 joints total)
- ✅ **Side-view display** with accurate forward kinematics
- ✅ **Interactive controls** - pause, step, speed control
- ✅ **Angle graph overlay** - visualize joint angles over time
- ✅ **Color-coded legs** - Red (right), Blue (left)
- ✅ **Live metrics** - frame info, angles, timing
- ✅ **Smooth playback** - adjustable speed (0.25x to 5x)

## 📦 Installation

### Requirements
```bash
pip install pygame
```

That's it! Just pygame is needed.

### Files
- `simulate_robot_leg.py` - Main simulator
- `output_angles.csv` - Your CSV data with joint angles

## 🚀 Quick Start

```bash
# Run with default settings (30 FPS)
python simulate_robot_leg.py

# Specify CSV file
python simulate_robot_leg.py --csv output_angles.csv

# Custom playback speed
python simulate_robot_leg.py --csv output_angles.csv --fps 50
```

## 🎮 Controls

| Key | Action |
|-----|--------|
| **SPACE** | Pause / Resume playback |
| **← →** | Step backward / forward one frame |
| **↑ ↓** | Increase / decrease playback speed |
| **R** | Reset to beginning |
| **G** | Toggle angle graph overlay |
| **Q / ESC** | Quit simulator |

## 📐 Robot Model

The simulator uses these segment lengths (scaled for visibility):

```
Hip → Knee:    60mm  (Thigh)
Knee → Ankle:  70mm  (Shin)
Ankle → Foot:  40mm  (Foot)
```

### Angle Conventions

- **Hip Angle**: Rotation of thigh relative to torso
  - 0° = straight down (vertical)
  - Positive = forward lean
  
- **Knee Angle**: Knee bend amount
  - 180° = straight leg
  - Less = bent (flexion)
  
- **Ankle Angle**: Foot orientation relative to shin
  - 90° = perpendicular
  - Varies for foot positioning

## 📊 CSV Format

Expected format from `output_angles.csv`:

```csv
timestamp,frame,right_hip_angle,right_knee_angle,right_ankle_angle,left_hip_angle,left_knee_angle,left_ankle_angle
2025-10-07 17:59:35.950,1,173.74,166.63,170.66,170.33,168.96,170.67
2025-10-07 17:59:36.110,2,175.37,169.67,172.09,171.79,169.68,170.53
...
```

**Required columns:**
- `right_hip_angle`, `right_knee_angle`, `right_ankle_angle`
- `left_hip_angle`, `left_knee_angle`, `left_ankle_angle`

**Optional columns:**
- `timestamp` - For display
- `frame` - Frame number

## 🎨 Visual Elements

### Display Components

1. **Ground Reference** - Gray horizontal line
2. **Grid Lines** - Vertical guides for alignment
3. **Torso** - Gray rectangle (body)
4. **Right Leg** - Red (hip → knee → ankle → foot)
5. **Left Leg** - Blue (hip → knee → ankle → foot)
6. **Joints** - Yellow circles at hip/knee/ankle
7. **Feet** - Larger colored circles
8. **HUD** - Frame info, angles, speed

### Angle Graph (Press G)

When enabled, shows real-time graph of all 6 joint angles:
- **Red shades** - Right leg joints
- **Blue shades** - Left leg joints
- **History** - Last 100 frames

## 🔧 Customization

### Adjust Robot Scale

Edit in `simulate_robot_leg.py`:
```python
SCALE_FACTOR = 2.5  # Increase for larger robot
```

### Change Colors

```python
COLORS = {
    'right_leg': (255, 100, 100),  # RGB for right leg
    'left_leg': (100, 100, 255),   # RGB for left leg
    ...
}
```

### Modify Segment Lengths

```python
SEGMENT_LENGTHS = {
    'hip_to_knee': 60,    # mm
    'knee_to_ankle': 70,  # mm
    'ankle_to_foot': 40   # mm
}
```

## 📈 Use Cases

### 1. Gait Analysis
Visualize walking patterns and identify irregularities:
```bash
python simulate_robot_leg.py --csv walking_data.csv --fps 30
```

### 2. Slow Motion Study
Examine joint movements frame-by-frame:
- Press **SPACE** to pause
- Use **← →** to step through frames
- Press **G** to see angle graphs

### 3. Speed Comparison
Test different playback speeds:
- Press **↑** to speed up (up to 5x)
- Press **↓** to slow down (down to 0.25x)

### 4. Animation Export (Future)
Record screen with OBS or similar tools while running simulator.

## 🐛 Troubleshooting

### Simulator won't start
```bash
# Check pygame installation
python -c "import pygame; print(pygame.version.ver)"

# Reinstall if needed
pip install --upgrade pygame
```

### CSV not loading
- Verify file path is correct
- Check CSV has required angle columns
- Ensure angles are numeric (not blank/text)

### Legs look weird
- Angles may need offset adjustment
- Check if your CSV angles are in degrees (not radians)
- Verify angle range is 0-360° or 0-180°

### Performance issues
- Reduce playback FPS: `--fps 20`
- Close angle graph (Press **G**)
- Reduce window size in code

## 🎯 Future Enhancements

Planned features (stretch goals):

- [ ] **3D Visualization** - PyOpenGL or VPython
- [ ] **Camera Controls** - Zoom, pan, rotate
- [ ] **Export Animation** - Save as MP4/GIF
- [ ] **Live Data Stream** - Real-time from OpenCV
- [ ] **Multiple Views** - Front, side, top simultaneously
- [ ] **Comparison Mode** - Overlay multiple CSV files
- [ ] **Ground Contact Detection** - Highlight when foot touches ground
- [ ] **Center of Mass** - Show balance point

## 📚 Technical Details

### Forward Kinematics

The simulator uses trigonometric forward kinematics to calculate joint positions:

```python
# For each segment:
x = previous_x + length * sin(angle)
y = previous_y + length * cos(angle)
```

Angles are chained from hip → knee → ankle → foot.

### Frame Timing

- **Display FPS**: 60 Hz (smooth rendering)
- **Playback FPS**: 30 Hz default (matches typical capture rate)
- **Speed Multiplier**: 0.25x to 5x (adjustable)

Frame advance calculated as:
```python
advance_time = 1.0 / (playback_fps * speed_multiplier)
```

### Coordinate System

- Origin: Top-left (Pygame standard)
- X-axis: Left → Right
- Y-axis: Top → Bottom
- Angles: Clockwise from vertical

## 🤝 Integration

### With Arduino Controller

1. Capture gait with MediaPose → `output_angles.csv`
2. Visualize in simulator → verify movements look natural
3. Send to Arduino → `python pose_to_arduino.py --csv output_angles.csv`
4. Compare physical robot with simulation

### With Gait Analyzer

Use simulation to validate gait analysis results before deploying to hardware.

## 📝 License

Part of bionic-leg-mediapose-pipeline project.

## 🙏 Credits

- **Pygame** - Visualization library
- **MediaPose** - Joint angle source data
- Built for bionic leg gait replication research

---

**Questions?** Check SIMULATION.md for requirements or open an issue.

**Tip**: Use **G** key to toggle the angle graph overlay for detailed joint angle analysis!
