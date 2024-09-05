import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as viper
from pychrono import irrlicht as chronoirr
import pychrono.sensor as sens

# Create Chrono system
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))  # Set gravity in the negative Z direction
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# Create ground body
ground_mat = chrono.ChContactMaterialNSC()
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, -1))  # Position the ground slightly below the origin
ground.SetFixed(True)  # Fix the ground in place
# Set ground texture for visualization
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)

# Create Viper rover
rover = viper.Viper(system)  # Instantiate the Viper rover
driver = viper.ViperDCMotorControl()  # Create a driver for the rover
rover.SetDriver(driver)  # Assign the driver to the rover

# Initialize rover position and orientation
init_pos = chrono.ChVector3d(0, 0.2, 0)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)
rover.Initialize(chrono.ChFramed(init_pos, init_rot))

# Create sensor manager and add sensors
manager = sens.ChSensorManager(system)
intensity = 1.0
manager.scene.AddPointLight(chrono.ChVector3f(2, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)

offset_pose = chrono.ChFramed(chrono.ChVector3d(1.0, 0, 1.45), chrono.QuatFromAngleAxis(.2, chrono.ChVector3d(0, 1, 0)))
cam = sens.ChCameraSensor(
    rover.GetChassis().GetBody(),
    15,  # update rate
    offset_pose,
    720,  # image width
    480,  # image height
    1.408  # field of view
)
cam.SetName("Third Person POV")
cam.PushFilter(sens.ChFilterVisualize(720, 480, "Viper Front Camera"))
manager.AddSensor(cam)

# Set the simulation time step
time_step = 1e-3

# Set simulation and render step variables
step_number = 0
render_step_size = 1.0 / 25  # FPS = 25
render_steps = math.ceil(render_step_size / time_step)

# Simulation loop
time = 0
while vis.Run():
    time += time_step  # Increment the simulation time
    steering = 0
    max_steering = math.pi / 6  # Maximum steering angle
    
    # Define the steering behavior over time
    if 2 < time < 7:
        steering = max_steering * (time - 2) / 5  # Gradually increase steering
    elif 7 < time < 12:
        steering = max_steering * (12 - time) / 5  # Gradually decrease steering
    
    driver.SetSteering(steering)  # Set the steering for the rover

    rover.Update()  # Update the rover's state

    # Update the sensor manager
    manager.Update()

    # Render the scene at specific intervals
    if (step_number % render_steps == 0):
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
    
    # Advance the simulation by one time step
    system.DoStepDynamics(time_step)

    # Increment the step number
    step_number += 1