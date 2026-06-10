"""
M113 tracked vehicle simulation — PyChrono 9.0.0, Irrlicht renderer.

System type  : ChSystemSMC (owned by veh.M113 wrapper)
Vehicle      : veh.M113 (tracked, SINGLE_PIN shoes, BDS driveline, SHAFTS engine)
Terrain      : RigidTerrain flat patch (SMC contact material)
Extra body   : Long box obstacle placed ahead of the vehicle to test mobility
Driver       : Hard-coded throttle=0.8, steering=0.0 throughout the run
Initial pos  : (-5, 0, 0.5) — updated from baseline (0, 0, 1.1)
Expected     : M113 accelerates forward (throttle 0.8), climbs or contacts the
               long box, demonstrating tracked mobility over an obstacle.
"""

import math
import os

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


# === Named constants ===
STEP_SIZE       = 5e-4          # physics time step (s)
SIM_END         = 20.0          # simulation duration (s)
RENDER_FPS      = 50.0
RENDER_STEPS    = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))  # precomputed once

# Vehicle spawn
INIT_LOC = chrono.ChVector3d(-5.0, 0.0, 0.5)   # updated per prompt
INIT_ROT = chrono.ChQuaterniond(1, 0, 0, 0)     # no rotation

# Long box obstacle dimensions and placement
BOX_LENGTH = 10.0   # long in X direction
BOX_WIDTH  = 2.5
BOX_HEIGHT = 0.4
BOX_X      = 5.0    # placed ahead of the vehicle (vehicle starts at x=-5)
BOX_Y      = 0.0
BOX_Z      = BOX_HEIGHT / 2.0   # resting on flat terrain at z=0

# Data paths (required for catalog vehicle truths — scored core)
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# === Vehicle setup ===
vehicle = veh.M113()
vehicle.SetContactMethod(chrono.ChContactMethod_SMC)
vehicle.SetChassisFixed(False)
vehicle.SetTrackShoeType(veh.TrackShoeType_SINGLE_PIN)
vehicle.SetDrivelineType(veh.DrivelineTypeTV_BDS)
vehicle.SetEngineType(veh.EngineModelType_SHAFTS)
vehicle.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
vehicle.SetBrakeType(veh.BrakeType_SIMPLE)
vehicle.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
vehicle.Initialize()

# Visualization types (set after Initialize)
vehicle.SetChassisVisualizationType(chrono.VisualizationType_MESH)
vehicle.SetSprocketVisualizationType(chrono.VisualizationType_MESH)
vehicle.SetIdlerVisualizationType(chrono.VisualizationType_MESH)
vehicle.SetRoadWheelVisualizationType(chrono.VisualizationType_MESH)
vehicle.SetTrackShoeVisualizationType(chrono.VisualizationType_MESH)

# === System & bodies (created by the veh.M113 wrapper) ===
sys = vehicle.GetSystem()               # cache: ChSystemSMC owned by the wrapper
chassis = vehicle.GetChassisBody()      # cache: fetched once, reused for reference
# Track shoes / sprockets / idlers: created internally by the M113 wrapper
# Terrain patch and box obstacle added below to the same sys

# Set collision system and solver (REQUIRED for tracked contact)
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
sys.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)

print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

# === Terrain ===
terrain = veh.RigidTerrain(sys)
patch_mat = chrono.ChContactMaterialSMC()   # SMC — must match vehicle contact method
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch_mat.SetYoungModulus(2e7)

patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM,
    200.0,   # terrain length (X)
    100.0    # terrain width (Y)
)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Long box obstacle ===
# A long box placed ahead of the vehicle to test M113 mobility
box_mat = chrono.ChContactMaterialSMC()
box_mat.SetFriction(0.8)
box_mat.SetRestitution(0.01)
box_mat.SetYoungModulus(2e7)

box_body = chrono.ChBodyEasyBox(
    BOX_LENGTH, BOX_WIDTH, BOX_HEIGHT,
    2000.0,      # density (kg/m³)
    True,        # create visual shape
    True,        # create collision shape
    box_mat
)
box_body.SetName("long_box_obstacle")
box_body.SetPos(chrono.ChVector3d(BOX_X, BOX_Y, BOX_Z))
box_body.SetFixed(True)   # fixed obstacle to test vehicle mobility
sys.AddBody(box_body)

# === Driver — hard-coded throttle=0.8 (scripted, scored core per prompt) ===
driver = veh.DriverInputs()
driver.m_steering = 0.0
driver.m_throttle = 0.8
driver.m_braking  = 0.0

# === Visualization — ChTrackedVehicleVisualSystemIrrlicht ===
vis = veh.ChTrackedVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("M113 Tracked Vehicle — Mobility Test")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(vehicle.GetVehicle())


# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        time = sys.GetChTime()   # cache: current sim time

        if step_number % RENDER_STEPS == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        # Synchronize subsystems (tracked vehicle: 2-arg, no terrain)
        terrain.Synchronize(time)
        vehicle.Synchronize(time, driver)
        vis.Synchronize(time, driver)

        # Advance subsystems
        terrain.Advance(STEP_SIZE)
        vehicle.Advance(STEP_SIZE)   # advances the wrapper-owned system
        vis.Advance(STEP_SIZE)


        step_number += 1
        realtime_timer.Spin(STEP_SIZE)

except (RuntimeError, ValueError) as exc:   # solver divergence / bad physics state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
