@echo off
REM Quick launcher for Robot Leg Simulator
REM Double-click this file to run the simulator

echo ====================================
echo   Bionic Leg Simulator Launcher
echo ====================================
echo.

REM Check if Python is installed
python --version >nul 2>&1
if errorlevel 1 (
    echo ERROR: Python not found!
    echo Please install Python from https://www.python.org/
    pause
    exit /b 1
)

REM Check if pygame is installed
python -c "import pygame" >nul 2>&1
if errorlevel 1 (
    echo pygame not found. Installing...
    pip install pygame
    if errorlevel 1 (
        echo ERROR: Failed to install pygame
        pause
        exit /b 1
    )
)

REM Check if CSV file exists
if not exist "output_angles.csv" (
    echo WARNING: output_angles.csv not found!
    echo.
    echo Please ensure your CSV file is in the same directory.
    echo Press any key to exit...
    pause >nul
    exit /b 1
)

echo Starting simulator...
echo.
echo Controls:
echo   SPACE     - Pause/Resume
echo   LEFT/RIGHT - Step frames
echo   UP/DOWN   - Speed control
echo   R         - Reset
echo   G         - Toggle graph
echo   Q         - Quit
echo.

REM Run the simulator
python simulate_robot_leg.py --csv output_angles.csv

echo.
echo Simulator closed.
pause
