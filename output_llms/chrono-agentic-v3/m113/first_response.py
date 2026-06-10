"""
M113 Tracked Vehicle Simulation — PyChrono 9.0.x / Irrlicht

Models the M113 armored personnel carrier driving on flat rigid terrain.
System: ChSystemSMC (owned by veh.M113 wrapper).
Key bodies: M113 chassis, track shoes (single-pin), road wheels, sprockets, idlers.
Expected behaviour: vehicle accelerates from rest, maintains steady forward speed
on flat rigid terrain; track shoes engage the drive sprockets and the chassis
translates in the positive X direction.
"""

import os
import math
import csv
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Constants ===
STEP_SIZE    = 5e-4        # physics time step (s)
SIM_END      = 20.0        # simulation duration (s)
RENDER_FPS   = 50.0        # frames per second for rendering
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))  # precomputed once

TERRAIN_LENGTH = 200.0    # terrain patch length (m) along X
TERRAIN_WIDTH  = 100.0    # terrain patch width (m)  along Y

INIT_LOC = chrono.ChVector3d(0, 0, 0.8)   # chassis init position (slightly above z=0)
INIT_ROT = chrono.QUNIT                   # heading along +X

# === Data paths — required truth components for all catalog vehicles ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

# === Vehicle ===
vehicle = veh.M113()
vehicle.SetContactMethod(chrono.ChContactMethod_SMC)   # M113 truth uses SMC
vehicle.SetChassisFixed(False)                          # MANDATORY — fixed chassis won't move
vehicle.SetTrackShoeType(veh.TrackShoeType_SINGLE_PIN)
vehicle.SetDrivelineType(veh.DrivelineTypeTV_BDS)
vehicle.SetEngineType(veh.EngineModelType_SHAFTS)
vehicle.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
vehicle.SetBrakeType(veh.BrakeType_SIMPLE)
vehicle.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
vehicle.Initialize()

# Visualization types — must be set after Initialize()
# In PyChrono 9.0.0 source build, VisualizationType_* lives in pychrono.vehicle
vehicle.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetTrackShoeVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSprocketVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetIdlerVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetRoadWheelVisualizationType(veh.VisualizationType_PRIMITIVES)

# === System & bodies (created by the veh.M113 wrapper) ===
sys     = vehicle.GetSystem()       # ChSystemSMC owned by the wrapper  # cache: fetched once
chassis = vehicle.GetChassisBody()  # main chassis rigid body            # cache: fetched once
# track shoes / sprockets / idlers / road wheels created inside the M113 wrapper

sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED — contact scene
sys.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)           # stable solver for tracked contact

print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

# === Terrain ===
terrain_mat = chrono.ChContactMaterialSMC()   # SMC — must match vehicle contact method
terrain_mat.SetFriction(0.8)
terrain_mat.SetRestitution(0.01)
terrain_mat.SetYoungModulus(2e7)

terrain = veh.RigidTerrain(sys)
patch = terrain.AddPatch(
    terrain_mat,
    chrono.CSYSNORM,
    TERRAIN_LENGTH,
    TERRAIN_WIDTH
)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Visualization ===
# Tracked vehicle uses ChTrackedVehicleVisualSystemIrrlicht (not the wheeled one).
# Irrlicht call order: configure window → Initialize() → add scene elements → AttachVehicle.
vis = veh.ChTrackedVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("M113 Tracked Vehicle")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()   # vehicle demos use AddLightDirectional, NOT AddTypicalLights
vis.AttachVehicle(vehicle.GetVehicle())

# === Driver ===
# ChInteractiveDriverIRR takes the visual system (not the vehicle).
# Built after vis so vis is fully initialized.
driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0
throttle_time = 1.0
braking_time  = 0.3
driver.SetSteeringDelta(1.0 / (RENDER_FPS * steering_time))
driver.SetThrottleDelta(1.0 / (RENDER_FPS * throttle_time))
driver.SetBrakingDelta(1.0 / (RENDER_FPS * braking_time))
driver.Initialize()

# === Review-only setup ===


# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number    = 0

try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        time = sys.GetChTime()

        # Throttled rendering: render once per frame, not once per physics step
        if step_number % RENDER_EVERY == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()


        # Full subsystem synchronize (tracked: 2-arg vehicle Synchronize, no terrain arg)
        driver.Synchronize(time)
        terrain.Synchronize(time)
        vehicle.Synchronize(time, driver_inputs)   # 2-arg for tracked vehicles
        vis.Synchronize(time, driver_inputs)


        # Advance all subsystems — vehicle.Advance steps the wrapper-owned system
        driver.Advance(STEP_SIZE)
        terrain.Advance(STEP_SIZE)
        vehicle.Advance(STEP_SIZE)
        vis.Advance(STEP_SIZE)

        step_number += 1
        realtime_timer.Spin(STEP_SIZE)

except (RuntimeError, ValueError) as exc:   # solver divergence / bad vehicle state
    import traceback; traceback.print_exc()
    raise
finally:
    pass
