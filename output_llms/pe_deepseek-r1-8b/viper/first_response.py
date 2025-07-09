import os
import math
import numpy as np
import pychrono as chrono
from pychrono import robot
from pychrono.robosimian import RoboSimian
from pychrono.vehicle import VehVehicle
from pychrono.irrlicht import ChVisualShapeBox, ChVisualShapeSphere
from pychrono.robot import ChCoordsysD

# Initialize the system with SMC contact model
my_system = chrono.ChSystemSMC()
chrono.SetChronoDataPath(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # Adjust path as needed

# Set gravity
my_system.SetGravity(chrono.ChVector3d(0, -9.81, 0))

# Create ground body
ground_body = chrono.ChBody()
ground_body.SetMass(1000000)  # Make it very heavy
ground_body.SetPos(chrono.ChVector3d(0, 0, 0))
ground_body.SetFixed(True)
my_system.Add(ground_body)

# Setup visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(my_system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Viper Rover Simulation')
vis.Initialize()
vis.AddSkyBox()
cam_pos = chrono.ChVector3d(0, 10, 10)
cam_rot = chrono.QuatFromAngleX(chrono.CH_PI / 2)  # Rotate 90 degrees
vis.AddCamera(cam_pos, cam_rot)
vis.AddTypicalLights()

# Create and add the Viper rover
rover_body = chrono.ChBody()
rover_body.SetMass(100)
rover_body.SetInertiaXX(chrono.ChVector3d(100, 100, 100))
rover_body.SetPos(chrono.ChVector3d(0, 0, 0))
rover_body.EnableCollision(True)
my_system.Add(rover_body)

# Add vehicle components
veh = VehVehicle()
veh.AddVehicleBody(rover_body)
veh.AddVehicleWheels(rover_body, 4)  # Four wheels
# Configure wheels
wheel_radius = 0.3
wheel_friction = 1.0
for i in range(4):
    wheel = veh.GetVehicleWheels()[i]
    wheel.SetRadius(wheel_radius)
    wheel.SetFriction(wheel_friction)
    wheel.SetDampingF(0.1)  # Damping coefficient

# Initialize the driver
driver = RoboSimian.RS_Driver(my_system, 'left', 'right', 'front', 'rear')
driver.SetInitialPosition(chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(chrono.CH_PI / 2)))  # Start upright

# Define steering function for the driver
steering_func = chrono.ChFunctionSine(0.01, 1.5)  # Sine wave with amplitude 1.5, frequency 0.01
driver.SetSpeedFunction(steering_func)

# Add visualization for the rover
rover_vis = chrono.ChVisualShapeBox(chrono.ChVector3d(rover_body.GetSizeX(), rover_body.GetSizeY(), rover_body.GetSizeZ()))
rover_body.AddVisualShape(rover_vis)

# Simulation loop
steering_angle = 0
time_step = 0.01

while True:
    # Update driver motion
    driver.UpdateDriverStep()
    
    # Update visualization
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    # Steer the rover
    my_system.DoStepDynamics(time_step)
    
    # Update camera position for a better view
    cam_pos = chrono.ChVector3d(0, 10, 10)
    cam_rot = chrono.QuatFromAngleX(chrono.CH_PI / 2)
    vis.AddCamera(cam_pos, cam_rot)
    
    # Check if the simulation should continue
    if not vis.Run():
        break