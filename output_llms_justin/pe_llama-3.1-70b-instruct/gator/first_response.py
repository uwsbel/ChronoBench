import os
import math
import numpy as np
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh

# Set the path to the Chrono data directory
chrono.SetChronoDataPath(os.path.join(os.path.dirname(__file__), "data"))

# Create the simulation system and visualization
system = chrono.ChSystemNSC()
visual_system = chronoirr.ChVisualSystemIrrlicht()
visual_system.SetWindowSize(1024, 768)
visual_system.SetWindowTitle("Gator Vehicle Simulation")
visual_system.SetCamera(chrono.ChVector3D(0, 0, 6), chrono.ChVector3D(0, 0, 0))

# Create the vehicle
gator = veh.Gator()
gator.SetContactMethod(chrono.ChContactMethod_NSC)
gator.SetChassisCollisionType(chrono.ChCollisionType_AABB)
gator.SetInitPosition(chrono.ChCoordsysD(chrono.ChVector3D(0, 0, 0), chrono.QUNIT))
gator.SetVisualizationType(chrono.ChVisualizationType_MESH)
gator.SetTireModel(chrono.ChTireModelType_TMEASY)
gator.Initialize(system)

# Create the terrain
terrain = veh.RigidTerrain(gator.GetSystem())
patch_mat = chrono.ChMaterialSurfaceNSC()
patch_mat.SetFriction(0.9)
patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysD(chrono.ChVector3D(0, 0, 0), chrono.QUNIT), 100, 100)
terrain.Initialize()

# Set up the driver system
driver = veh.ChIrrGuiDriver()
driver.SetVehicle(gator)
driver.Initialize(visual_system)

# Simulation loop
steering = 0
throttle = 0
braking = False
while visual_system.Run():
    driver.Synchronize(True)
    system.DoStepDynamics(0.02)
    visual_system.BeginScene(True, True, chronoirr.SColor(255, 255, 255))
    visual_system.Render()
    driver.AddShadowAll()
    visual_system.EndScene()
    driver.SetSteering(steering)
    driver.SetThrottle(throttle)
    driver.SetBraking(braking)
    if driver.IsKeyDown(chronoirr.KEYBOARD_KEY_UPARROW):
        throttle += 0.01
    if driver.IsKeyDown(chronoirr.KEYBOARD_KEY_DOWNARROW):
        throttle -= 0.01
    if driver.IsKeyDown(chronoirr.KEYBOARD_KEY_LEFTARROW):
        steering -= 0.01
    if driver.IsKeyDown(chronoirr.KEYBOARD_KEY_RIGHTARROW):
        steering += 0.01
    if driver.IsKeyDown(chronoirr.KEYBOARD_KEY_SPACE):
        braking = not braking