import os
import math
import numpy as np
import pychrono as chrono
from pychrono import robot
from pychrono.terrain import veh
from pychrono.robots import HMMWV
from pychrono.irrlicht import chronoirr
from pychrono.vehicle import vehicle

# Initialize the PyChrono system with NSC mechanics
sys = chrono.ChSystemNSC()
chrono.SetChronoDataPath("data")  # Set the data path

# Initialize the vehicle
hmmvw = HMMWV(sys)
hmmvw.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(chrono.CH_PI)))  # Initial position and orientation
hmmvw.SetContactMethod("hinge")  # Set contact method to hinge
hmmvw.SetTireModel("TMEASY")  # Use TMEASY tire model

# Initialize visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.AddLogo(chrono.GetChronoDataFile("pychrono.png"))
vis.AddSkyBox()
cam_pos = chrono.ChVector3d(5, 5, 10)
vis.AddCamera(cam_pos)
vis.Initialize()

# Create and attach the terrain
terrain = veh.RigidTerrain(sys)
# Add collision mesh
collision_mesh = terrain.AddCollisionMesh("Highway_col.obj")
terrain.Initialize(collision_mesh)
# Add visual mesh
visual_mesh = terrain.AddVisualMesh("Highway_vis.obj")
terrain.GetVisuals().SetMaterial(visual_mesh, chrono.ChColor(1, 0.5, 0.5))  # Set visual color

# Attach vehicle to terrain
terrain_link = chrono.ChLinkLockPrismatic()
terrain_link.Initialize(terrain, hmmvw, chrono.ChCoordsysd(chrono.ChVector3d(0, -5, 0), chrono.QuatFromAngleX(chrono.CH_PI/2)))
sys.Add(terrain_link)

# Set up driver controls
driver = vehicle.RSDriver(sys, "driver")
driver.SetSteeringFunction(chrono.ChFunction_Sine(0.1, 1.0))  # Steering control
driver.SetThrottleFunction(chrono.ChFunction_Sine(0.1, 1.0))  # Throttle control
driver.SetBrakeFunction(chrono.ChFunction_Sine(0.1, 1.0))    # Brake control
driver.EnableDriver(True)

# Add vehicle to system
sys.Add(hmmvw)

# Set up simulation loop
fps = 50  # 50 frames per second
time_step = 1.0 / fps  # Time step in seconds

# Custom contact reporting callback (optional)
def OnReportContact(self, pA, pB, plane_coord, distance, eff_radius, cforce, ctorque, modA, modB):
    print(f"Contact detected between body A and B at point {pA}")

# Register the callback if needed
sys.GetContactContainer().RegisterAllContactCallbacks(OnReportContact)

# Main simulation loop
while True:
    # Update driver controls
    driver.Update()
    
    # Perform simulation step
    sys.DoStepDynamics(time_step)
    
    # Render the scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    # Check if the window should close
    if vis.WindowCloseRequested():
        break

# Terminate the visualization
vis.Terminate()