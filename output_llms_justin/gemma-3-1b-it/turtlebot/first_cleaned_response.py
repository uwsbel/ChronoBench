import pychrono
import time
import random
import numpy as np
from pychrono.simulation import Simulation
from pychrono.visualize import Irrlicht


SIM_NAME = "TurtlebotSimulation"
GRAVITY = 9.81
GROUND_BODY_MASS = 1000  
GROUND_BODY_RADIUS = 0.5  
GROUND_BODY_AREA = np.pi * (GROUND_BODY_RADIUS**2)
GROUND_BODY_CENTER_X = 0.0
GROUND_BODY_CENTER_Y = 0.0
INITIAL_POSITION = (10, 10)
INITIAL_ORIENTATION = 0.0  
SIM_TIME_STEP = 0.1  
SIM_DURATION = 60  


simulation = Simulation(SIM_NAME)


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



irrlicht = Irrlicht()
irrlicht.initialize(800, 600)
irrlicht.color = (1, 1, 1)  
irrlicht.set_camera_position(100, 100, 200)  
irrlicht.set_camera_brightness(1.0)
irrlicht.set_camera_fov(45)
irrlicht.set_camera_location(100, 100)
irrlicht.set_camera_orientation(0, 0, 0)  
irrlicht.set_background_color(0)
irrlicht.set_title("Turtlebot Simulation")


def set_motor_speeds(left_speed, right_speed):
    print(f"Setting left motor speed: {left_speed}, right motor speed: {right_speed}")
    
    
    time.sleep(0.1)  
    left_speed = left_speed * 0.5  
    right_speed = right_speed * 0.5


def update_turtlebot():
    left_speed = 0
    right_speed = 0
    
    
    time.sleep(SIM_TIME_STEP)
    left_speed = 0.5 * (1 - left_speed)
    right_speed = 0.5 * (1 - right_speed)
    
    
    turtlebot.set_position(INITIAL_POSITION + (left_speed, right_speed))


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