import pychrono
import time
import random
import numpy as np
from pychrono.simulation import Simulation
from pychrono.visualize import Irrlicht

# Simulation Parameters
SIM_NAME = "TurtlebotSimulation"
GRAVITY = 9.81
GROUND_BODY_MASS = 1000  # kg
GROUND_BODY_RADIUS = 0.5  # m
GROUND_BODY_AREA = np.pi * (GROUND_BODY_RADIUS**2)
GROUND_BODY_CENTER_X = 0.0
GROUND_BODY_CENTER_Y = 0.0
INITIAL_POSITION = (10, 10)
INITIAL_ORIENTATION = 0.0  # Degrees
SIM_TIME_STEP = 0.1  # Time steps for simulation
SIM_DURATION = 60  # Seconds

# Create Simulation
simulation = Simulation(SIM_NAME)

# Initialize the Turtlebot
try:
    turtlebot = simulation.create_turtlebot(
        initial_position=INITIAL_POSITION,
        initial_orientation=INITIAL_ORIENTATION,
        mass=GROUND_BODY_MASS,
        radius=GROUND_BODY_RADIUS,
        area=GROUND_BODY_AREA,
        center_x=GROUND_BODY_CENTER_X,
        center_y=GROUND_BODY_CENTER_Y
    )
except Exception as e:
    print(f"Error creating Turtlebot: {e}")
    exit()


# Create Irrlicht visualization
irrlicht = Irrlicht()
irrlicht.initialize(800, 600)
irrlicht.color = (1, 1, 1)  # White
irrlicht.set_camera_position(100, 100, 200)  # Camera position
irrlicht.set_camera_brightness(1.0)
irrlicht.set_camera_fov(45)
irrlicht.set_camera_location(100, 100)
irrlicht.set_camera_orientation(0, 0, 0)  # Rotate the camera
irrlicht.set_background_color(0)
irrlicht.set_title("Turtlebot Simulation")

# Set up the Turtlebot's motors
def set_motor_speeds(left_speed, right_speed):
    print(f"Setting left motor speed: {left_speed}, right motor speed: {right_speed}")
    # Simulate motor control (replace with actual motor control logic)
    # This is a placeholder for a simple simulation
    time.sleep(0.1)  # Simulate time for the motor control
    left_speed = left_speed * 0.5  # Example:  Reduce speed
    right_speed = right_speed * 0.5

# Update the Turtlebot's position
def update_turtlebot():
    left_speed = 0
    right_speed = 0
    
    # Simulate a simple movement pattern
    time.sleep(SIM_TIME_STEP)
    left_speed = 0.5 * (1 - left_speed)
    right_speed = 0.5 * (1 - right_speed)
    
    # Update the Turtlebot's position
    turtlebot.set_position(INITIAL_POSITION + (left_speed, right_speed))

# Simulation Loop
try:
    while True:
        update_turtlebot()
        irrlicht.clear()
        irrlicht.display(turtlebot)
        irrlicht.close()
        time.sleep(SIM_DURATION)

except KeyboardInterrupt:
    print("Simulation stopped by user.")
    irrlicht.close()
    simulation.destroy()
    exit()