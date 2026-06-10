"""
Viper rover on SCM deformable terrain — turn 3.

Changes from turn 2 (rigid ground + time-based steering):
  - Import pychrono.vehicle as veh
  - Replaced rigid ground body with SCM deformable terrain
  - Removed time-based steering: steering fixed at 0.0 (straight drive)

System: ChSystemNSC with Bullet collision
Terrain: SCM (Bekker-Janosi soft soil)
Rover:   Viper 6-wheel rover with DC-motor steering driver
"""

import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as robot
import pychrono.vehicle as veh
from pychrono import irrlicht as chronoirr

# === System & gravity ===
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# === SCM deformable terrain ===
terrain = veh.SCMTerrain(system)
terrain.SetPlane(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, -0.5)))
length = 14
width = 4
mesh_resolution = 0.02
terrain.Initialize(length, width, mesh_resolution)
terrain.SetSoilParameters(
    0.2e6,   # Bekker Kphi
    0,       # Bekker Kc
    1.1,     # Bekker n exponent
    0,       # Mohr cohesive limit (Pa)
    30,      # Mohr friction limit (degrees)
    0.01,    # Janosi shear coefficient (m)
    4e7,     # Elastic stiffness (Pa/m)
    3e4      # Damping (Pa s/m)
)
terrain.SetPlotType(veh.SCMTerrain.PLOT_PRESSURE, 0, 20000)
terrain.SetMeshWireframe(True)

# === Viper rover ===
rover = robot.Viper(system)
driver = robot.ViperDCMotorControl()
rover.SetDriver(driver)

init_pos = chrono.ChVector3d(0, 0.2, 0)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)
rover.Initialize(chrono.ChFramed(init_pos, init_rot))

# === Visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Viper rover - SCM terrain")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 2.5, 1.5), chrono.ChVector3d(0, 0, 1))
vis.AddTypicalLights()
vis.AddLightWithShadow(
    chrono.ChVector3d(1.5, -2.5, 5.5), chrono.ChVector3d(0, 0, 0.5),
    3, 4, 10, 40, 512
)

# === Main loop ===
time_step = 1e-3
sim_end = 20.0
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))

frame = 0
while vis.Run() and system.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        # Steering fixed at 0.0 (straight drive) — per turn 3 input
        driver.SetSteering(0.0)
        rover.Update()
        system.DoStepDynamics(time_step)
        if system.GetChTime() >= sim_end:
            break
