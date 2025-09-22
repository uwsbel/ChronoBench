import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as crobot
from pychrono import irrlicht as chronoirr

def main():
    # 1. Create the Chrono simulation system
    # Initialize the Chrono physical system that will manage all simulation components.
    system = chrono.ChSystemNSC()
    
    # Define contact material properties for collision elements
    # Initialize contact material with specified friction and restitution properties.
    phys_mat = chrono.ChContactMaterialNSC()
    phys_mat.SetFriction(0.9)  # Set friction coefficient to 0.9
    phys_mat.SetRestitution(0.01)  # Set restitution (bounciness) to 0.01

    # Create the ground body with specified dimensions and properties
    # Add a large ground volume to serve as the terrain for the simulation.
    ground = chrono.ChBodyEasyBox(20, 20, 1,  # Dimensions: 20x20x1 meters
                                  phys_mat,   # Use the defined contact material
                                  1000,       # Set density to 1000 kg/m^3
                                  True,       # Enable visualization
                                  True)       # Enable collision detection
    # Set ground position and orientation
    ground.SetPos(chrono.ChVector3d(0, 0, -1))  # Position the ground slightly below the origin
    ground.SetRot(chrono.QUNIT)  # No rotation (identity quaternion)
    ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
    # Apply concrete texture to the ground's visual shape
    ground.SetFixed(True)  # Fix the ground in place (no movement)
    system.Add(ground)  # Add the ground to the simulation system

    # Create the Curiosity rover and add it to the simulation
    # Instantiate the CuriosityMarsRover robot model
    rover = crobot.CuriosityMarsRover(system)
    rover.Initialize()  # Initialize the rover (set initial conditions, create internal components)

    # Create and configure the interactive driver for the rover
    # Instantiate the motor control driver for interactive steering
    driver = crobot.CuriosityMotorControl()
    rover.SetDriver(driver)  # Assign the driver to the rover

    # Initialize the Irrlicht visualization system
    # Create the Irrlicht visualization system for rendering the simulation
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)  # Attach the Chrono system to the visualization
    vis.SetCameraVertical(chrono.CameraVerticalDir.Z)  # Set camera orientation to vertical (Z-axis)
    vis.SetWindowSize(1280, 720)  # Set the window size for the visualization (1280x720 pixels)
    vis.SetWindowTitle('Curiosity rover - Rigid terrain')  # Set the title of the visualization window
    vis.Initialize()  # Initialize the visualization system
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))  # Add a logo to the visualization
    vis.AddSkyBox()  # Add a skybox for background
    vis.AddCamera(chrono.ChVector3d(0, 3, 3), chrono.ChVector3d(0, 0, 0))  # Add and position the camera
    vis.AddTypicalLights()  # Add typical lights for better visualization
    vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5), chrono.ChVector3d(0, 0, 0), 3, 4, 10, 40, 512)  # Add light with shadow
    vis.EnableShadowPlots()  # Enable shadows in the visualization

    # Set the simulation time step
    time_step = 1e-3  # Define the time step for the simulation (0.001 seconds)

    # Initialize time tracking variables
    time = 0
    time_limit = 30  # Set the total simulation time limit (30 seconds)

    # Start the interactive simulation loop
    vis.BeginScene()  # Begin the visualization scene
    vis.Render()  # Render the current scene
    vis.EndScene()  # End the visualization scene

    while vis.Run():
        time = rover.GetChTime()  # Update the simulation time based on the rover's time counter

        # End simulation after the specified time limit
        if time > time_limit:
            break

        # Apply interactive driver inputs to the rover
        driver.SetSteering(0.3 * math.sin(0.2 * time))  # Set steering input with a sinusoidal pattern
        rover.Update()  # Update the rover's state based on the inputs

        # Render the scene
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        # Advance the simulation by one time step
        system.DoStepDynamics(time_step)

if __name__ == "__main__":
    main()