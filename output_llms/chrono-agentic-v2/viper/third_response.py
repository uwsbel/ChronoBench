"""
Viper rover simulation on SCM deformable terrain.

System type: ChSystemNSC with Bullet collision.
Main bodies: Viper rover (chassis + wheels via robot.Viper), SCM soft-soil terrain.
Driver: ViperDCMotorControl with constant steering = 0.0 (straight forward).
Expected behavior: Rover drives straight forward, sinking into and leaving ruts
in the deformable SCM terrain surface.
"""

import os
import math
import pychrono.core as chrono
import pychrono.robot as robot
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Simulation parameters ===
TIME_STEP   = 1e-3      # physics time step (s)
SIM_END     = 10.0      # simulation end time (s)
RENDER_FPS  = 50.0      # review-video frame rate
render_every = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

# SCM terrain geometry
SCM_SIZE_X  = 14.0      # terrain length (m)
SCM_SIZE_Y  = 4.0       # terrain width (m)
SCM_DELTA   = 0.02      # mesh resolution (m)
SCM_PLANE_Z = -0.5      # terrain reference plane offset

# Rover spawn
ROVER_SPAWN = chrono.ChVector3d(0, 0, 0.2)

# === System setup ===
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# === SCM deformable terrain ===
terrain = veh.SCMTerrain(system)
terrain.SetPlane(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, SCM_PLANE_Z)))
terrain.Initialize(SCM_SIZE_X, SCM_SIZE_Y, SCM_DELTA)
terrain.SetSoilParameters(0.2e6, 0, 1.1, 0, 30, 0.01, 4e7, 3e4)
terrain.SetPlotType(veh.SCMTerrain.PLOT_PRESSURE, 0, 20000)
terrain.SetMeshWireframe(True)

# === Viper rover ===
rover  = robot.Viper(system)
driver = robot.ViperDCMotorControl()
rover.SetDriver(driver)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)  # identity
rover.Initialize(chrono.ChFramed(ROVER_SPAWN, init_rot))

# cache chassis body for sensor/logging access
chassis_body = rover.GetChassis().GetBody()  # cache: fetched once, reused every step

# === Visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Viper rover - SCM deformable terrain")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 2.5, 1.5), chrono.ChVector3d(0, 0, 0.5))
vis.AddTypicalLights()
vis.AddLightWithShadow(
    chrono.ChVector3d(1.5, -2.5, 5.5),
    chrono.ChVector3d(0, 0, 0.5),
    3, 4, 10, 40, 512,
)

# === Review-only recording setup ===

# === Main loop ===
frame = 0
try:
    while vis.Run() and system.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            time = system.GetChTime()  # cache: current sim time for this step
            driver.SetSteering(0.0)
            rover.Update()
            system.DoStepDynamics(TIME_STEP)
            if system.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / bad state
    import traceback; traceback.print_exc()
    raise
