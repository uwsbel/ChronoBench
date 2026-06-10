"""
HMMWV vehicle simulation with ROS integration (SimBench vehros turn 1).
Models an HMMWV on rigid terrain with ROS clock, driver-inputs, and vehicle-state handlers.
"""

import os
import math
import csv
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.ros as chros
import pychrono.irrlicht as chronoirr

# === Review-only recording scaffolding ===


# === Named constants ===
TIME_STEP = 1e-3
SIM_END = 10.0
RENDER_FPS = 50.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))

TERRAIN_FRICTION = 0.9
TERRAIN_RESTITUTION = 0.01
TERRAIN_SIZE = 100.0

VEHICLE_INIT_POS = chrono.ChVector3d(0, 0, 1.6)
VEHICLE_INIT_ROT = chrono.ChQuaterniond(1, 0, 0, 0)

# === Data paths ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

# === Create HMMWV vehicle ===
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysd(VEHICLE_INIT_POS, VEHICLE_INIT_ROT))
hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)
hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
hmmwv.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)
hmmwv.SetTireType(veh.TireModelType_TMEASY)
hmmwv.SetTireStepSize(TIME_STEP)
hmmwv.Initialize()

print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())

system = hmmwv.GetSystem()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# === Create terrain ===
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(TERRAIN_FRICTION)
patch_mat.SetRestitution(TERRAIN_RESTITUTION)
terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_SIZE, TERRAIN_SIZE)
terrain.Initialize()

# === Create driver system ===
driver = veh.ChDriver(hmmwv.GetVehicle())
driver.Initialize()

# === ROS manager and handlers ===
ros_manager = chros.ChROSPythonManager()
ros_manager.RegisterHandler(chros.ChROSClockHandler())
ros_manager.RegisterHandler(
    chros.ChROSDriverInputsHandler(25, driver, "~/input/driver_inputs")
)
ros_manager.RegisterHandler(
    chros.ChROSBodyHandler(25, hmmwv.GetChassisBody(), "~/output/hmmwv/state")
)
ros_manager.Initialize()

# === Visualization (full Irrlicht block — built unconditionally) ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("HMMWV ROS Simulation")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataPath() + "logo_pychrono_alpha.png")
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, -12, 6), chrono.ChVector3d(0, 0, 1.5))
vis.AddLightDirectional()
vis.AddGrid(1.0, 1.0, 40, 40, chrono.ChCoordsysd(), chrono.ChColor(0.4, 0.4, 0.4))

# === CSV logging setup (review-only) ===

# === Main simulation loop ===
hmmwv.GetVehicle().EnableRealtime(True)

frame = 0

while vis.Run() and system.GetChTime() < SIM_END:
    # Throttled rendering
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Review-only frame capture

    # Physics batch
    for _ in range(RENDER_EVERY):
        sim_time = system.GetChTime()

        driver_inputs = driver.GetInputs()
        driver.Synchronize(sim_time)
        terrain.Synchronize(sim_time)
        hmmwv.Synchronize(sim_time, driver_inputs, terrain)

        driver.Advance(TIME_STEP)
        terrain.Advance(TIME_STEP)
        hmmwv.Advance(TIME_STEP)

        if not ros_manager.Update(sim_time, TIME_STEP):
            break

        # Review-only CSV logging

        if system.GetChTime() >= SIM_END:
            break

# === Post-loop review-only cleanup and assembly ===

# === Post-processing (review-only) ===

print("Simulation complete.")
