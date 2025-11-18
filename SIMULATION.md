I want you to build a visual simulation of my robot's leg movement using the same CSV file that contains joint angles from OpenCV.

Below is exactly what you need to know and what I want you to build.

📌 Goal

Create a real-time visualizer that shows both legs (6 joints total) moving according to:

timestamp,frame,right_hip_angle,right_knee_angle,right_ankle_angle,left_hip_angle,left_knee_angle,left_ankle_angle


This is not for motor control — only visual simulation.

🎯 MVP Requirements

Build a simple but accurate simulation using Python.

MVP features:

Use Matplotlib, PyGame, or PyOpenGL to display:

Two legs (left + right)

Each leg with 3 joints (hip → knee → ankle)

Read CSV rows sequentially.

For each frame:

Convert joint angles into line segments (forward kinematics).

Draw in 2D (side view) or 3D (optional).

Play frames at real-time speed (based on timestamps or fixed FPS).

Ability to pause, resume, step frame, and reset.

🦾 Robot Model Specs (Use These)

Leg = 3 segments:

hip → knee = 60mm
knee → ankle = 70mm
ankle → foot = 40mm  (optional)


Assume 2D sagittal plane (side-view) for MVP.

📐 Angle Interpretation

Angles from CSV are in degrees.

Define conventions:

hip_angle   = rotation of thigh relative to torso (0° = vertical)
knee_angle  = bend amount (like human knee, flexion)
ankle_angle = angle of foot relative to tibia


It’s fine if the angles need small offset adjustments — just make movement visually correct.

🎨 Simulation Features for MVP

Draw torso as fixed point or vertical line.

Draw two legs that move independently.

Draw joints as circles.

Draw bones as lines.

Update at 30 FPS.

Interpolate between frames if CSV frame rate differs.

🚀 Stretch Features (if easily possible)

Add option for 3D visualization using PyOpenGL or VPython.

Add on-screen graphs of angles changing over time.

Allow importing live data stream instead of CSV.

Add camera controls (zoom/pan).

Export animation as MP4.

🤖 Deliverables

Claude, please generate:

1. simulate_robot_leg.py

A full Python script that:

Loads CSV.

Plays simulation in real-time.

Shows the side-view of robot legs.

Has keyboard controls:

Space → pause/play

Left/Right arrow → step frames

R → reset

Q → quit

2. Any helper files if needed.
3. Clean, readable, commented code.

Important:
Make this runnable with only standard Python + commonly available graphics library (choose one: matplotlib animation / pygame / vpython).