import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as viper
from pychrono import irrlicht as chronoirr
import pychrono.sensor as sens # MODIFICATION 1: Import sensor

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
ground.SetBodyFixed(True)  # Fix the ground in place - ERROR CORRECTION (was SetFixed)
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

# MODIFICATION 2: Creation of a sensor manager and add a point light
manager = sens.ChSensorManager(system)
intensity = 1.0
# Add a point light to the scene for the sensor
manager.scene.AddPointLight(chrono.ChVector3f(2, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
# Optional: Set a background color for the sensor simulation if needed
# manager.scene.GetBackground().color = chrono.ChColor(0.2, 0.3, 0.4)


# MODIFICATION 3: Creation of a camera sensor
# Camera offset pose relative to the rover's chassis body
offset_pose = chrono.ChFramed(chrono.ChVector3d(1.0, 0, 1.45), chrono.QuatFromAngleAxis(0.2, chrono.ChVector3d(0, 1, 0)))
cam = sens.ChCameraSensor(
    rover.GetChassis().GetBody(),  # parent body
    15,                            # update rate in Hz
    offset_pose,                   # offset pose from parent
    720,                           # image width
    480,                           # image height
    1.408                          # horizontal field of view (radians)
)
cam.SetName("Third Person POV")
# Add a filter to visualize the camera data in a dedicated window
cam.PushFilter(sens.ChFilterVisualize(720, 480, "Viper Front Camera"))
manager.AddSensor(cam)


# Create run-time visualization (Irrlicht)
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Viper rover - Rigid terrain')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 2.5, 1.5), chrono.ChVector3d(0, 0, 1)) # Main Irrlicht camera
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5), chrono.ChVector3d(0, 0, 0.5), 3, 4, 10, 40, 512)

# Enable shadows (commented out to improve performance in original, kept as is)
# vis.EnableShadows()

# Set the simulation time step
time_step = 1e-3

# MODIFICATION 4: Addition of variables for simulation step and render step size
step_number = 0
render_fps = 25  # Desired render FPS for Irrlicht visualization
render_step_size = 1.0 / render_fps
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

    rover.Update()  # Update the rover's state (processes driver inputs, updates powertrain)

    # MODIFICATION 5: Modification of the simulation loop to render the scene at specific intervals
    # Render the Irrlicht scene at specific intervals
    if vis.Run() and (step_number % render_steps == 0): # Check vis.Run() again in case window was closed
        vis.BeginScene(True, True, chrono.ChColor(0.0,0.0,0.0)) # Clear with black or any other color
        vis.Render()
        vis.EndScene()
    
    # Advance the simulation by one time step
    system.DoStepDynamics(time_step)

    # MODIFICATION 5: Update the sensor manager
    manager.Update() # Update all sensors in the manager

    step_number += 1

# Simulation ended (Irrlicht window closed)
# No explicit cleanup needed for basic manager and vis unless specific resources were opened (e.g. file writers for sensors)