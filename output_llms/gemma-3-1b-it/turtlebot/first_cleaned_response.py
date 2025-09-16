import pychrono
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import time


simulation_time = 60  
dt = 0.1  
gravity = 9.81
ground_body_x = 0
ground_body_y = 0
turtlebot_x = 0
turtlebot_y = 0
velocity_x = 0
velocity_y = 0
angle_x = 0
angle_y = 0
rotation_speed = 0.5  


chrono = pychrono.Chrono()


turtlebot = chrono.Turtlebot(
    x=turtlebot_x,
    y=turtlebot_y,
    radius=0.5,
    speed=1,
    orientation=chrono.Orientation.RIGHT,
    color="white",
    texture="white"
)


turtlebot.set_position(ground_body_x, ground_body_y)
turtlebot.set_orientation(chrono.Orientation.RIGHT)
turtlebot.set_translation(ground_body_x, ground_body_y, 0)


ground_body = chrono.GroundBody(
    x=0,
    y=0,
    radius=0.5,
    color="brown",
    texture="brown"
)


camera = chrono.Camera(
    x=0,
    y=0,
    width=50,
    height=50,
    color="white",
    texture="white"
)


def update_animation(frame_num):
    
    global turtlebot, ground_body, camera

    
    turtlebot.set_position(turtlebot.position[0], turtlebot.position[1], 0)
    turtlebot.set_orientation(chrono.Orientation.RIGHT)
    turtlebot.set_translation(turtlebot.position[0], turtlebot.position[1], 0)

    
    camera.set_position(turtlebot.position[0], turtlebot.position[1], 0)

    
    plt.clf()  
    plt.imshow(camera.image, cmap='gray')
    plt.pause(0.01)

    return camera


for frame_num in range(simulation_time):
    update_animation(frame_num)
    time.sleep(dt)  


turtlebot.set_translation(0, 0, 0)
turtlebot.set_orientation(chrono.Orientation.RIGHT)
turtlebot.set_position(0, 0)
turtlebot.set_translation(0, 0, 0)
turtlebot.set_orientation(chrono.Orientation.RIGHT)
turtlebot.set_translation(0, 0, 0)
turtlebot.set_position(0, 0, 0)
turtlebot.set_orientation(chrono.Orientation.RIGHT)
turtlebot.set_translation(0, 0, 0)
turtlebot.set_rotation(chrono.Rotation.NONE) 
plt.close()