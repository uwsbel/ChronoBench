import pychrono
import pychrono.core as pc
import pychrono.visual as v
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# 1. Initialize PyChrono Environment
try:
    chrono = pc.chrono.Chrono()
except Exception as e:
    print(f"Error initializing Chrono: {e}")
    exit()

# 2. Define Physical Systems and Objects
# Kraz Vehicle
vehicle = pc.Vehicle(
    x=0.0,
    y=0.0,
    vx=0.0,
    vy=0.0,
    mass=10.0,  # kg
    friction=0.8,
    restitution=0.7,
    initial_position=(0.0, 0.0) # Initial position
)

# Terrain
terrain = pc.Terrain(
    x_min=-10.0,
    x_max=10.0,
    y_min=-10.0,
    y_max=10.0,
    friction=0.9,
    restitution=0.8,
    initial_position=(0.0, 0.0)
)

# Driver System
driver = pc.Driver(
    vehicle=vehicle,
    initial_position=(0.0, 0.0),
    control_method="manual", # Can be "manual", "automatic", or "hybrid"
    max_speed=5.0, # m/s
    acceleration=0.1, # m/s^2
    deceleration=0.1, # m/s^2
    max_deceleration=1.0, # m/s^2
    simulation_time=10.0 # seconds
)

# 3. Set Default Parameters
chrono.set_default_parameters(
    time_step=0.01, # seconds
    initial_velocity=0.0, # m/s
    gravity=9.81,
    friction=0.8,
    restitution=0.7
)

# 4. Visualization Setup
fig, ax = v.figure(width=800, height=600)
scene = v.Scene()

# 5. Initialize Irrlicht Camera
camera = v.Camera(
    x=0.0,
    y=0.0,
    z=0.0,
    width=800,
    height=600,
    fov=60.0,
    aspect=2.0,
    near=-10.0,
    far=10.0,
    color=(1.0, 1.0, 1.0),  # White
    animation=True
)

# 6. Initialize Animation Function
def animate(i):
    camera.set_position(camera.position)
    camera.set_rotation(camera.rotation)
    
    # Update the camera position for each frame
    camera.update()
    
    return scene.animate(camera,  # Return the animation function
                     lambda i=i:  # Pass the current frame number to the function
                     animate(i))

# 7. Create the Animation
ani = v.FuncAnimation(fig, animate, frames=chrono.get_frame_count(),
                      interval=10,
                      repeat=False)

# 8. Display the Visualization
v.show(fig, scene, title="Kraz Simulation")

# 9.  Save the Animation (optional)
chrono.save_animation(ani, "kraz_simulation.mp4")
print("Simulation complete. Animation saved to kraz_simulation.mp4")