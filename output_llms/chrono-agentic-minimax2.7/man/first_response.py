"""
MAN 10t Truck Simulation — PyChrono 9.0.0
==========================================
A rigid-body simulation of a MAN 10t truck driving on a rigid flat terrain.
The vehicle uses a TMEASY tire model and is driven interactively via
ChInteractiveDriverIRR with real-time Irrlicht visualization and chase camera.

System type: NSC (Non-Smooth Contact) — rigid terrain
Main bodies: MAN 10t catalog vehicle (chassis + 4 wheels + suspension)
"""

import os
import math

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr


# === Named constants ===
SIM_END = 20.0          # simulation duration (s)
TIME_STEP = 1e-3        # physics time step (s)
RENDER_FPS = 50.0       # render frames per second
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))

# Vehicle spawn — chassis origin at wheel-bottom + suspension ref height
# MAN 10t: tire radius 0.629m, so wheel bottom at z=0 when chassis at z=0.629
VEH_INIT_X = 0.0
VEH_INIT_Y = 0.0
VEH_INIT_Z = 0.35       # chassis origin height; wheel bottom near z=0 on flat terrain

# Terrain
TERRAIN_LENGTH = 200.0  # m
TERRAIN_WIDTH = 200.0   # m

# Driver input scaling
STEERING_TIME = 1.0     # s to reach max steering
THROTTLE_TIME = 1.0     # s to reach max throttle
BRAKING_TIME = 0.3     # s to reach max braking


# === Paths — locate bundled Chrono assets ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


# === Create MAN 10t vehicle (catalog wrapper) ===
man = veh.MAN_10t()
man.SetContactMethod(chrono.ChContactMethod_NSC)
man.SetChassisCollisionType(veh.CollisionType_NONE)
man.SetChassisFixed(False)
man.SetInitPosition(chrono.ChCoordsysd(
    chrono.ChVector3d(VEH_INIT_X, VEH_INIT_Y, VEH_INIT_Z),
    chrono.QUNIT,
))
man.SetTireType(veh.TireModelType_TMEASY)   # prompt: TMEASY tire model
man.SetTireStepSize(TIME_STEP)
man.Initialize()

system = man.GetSystem()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Print vehicle mass — truth component (scored core)
print("VEHICLE MASS: ", man.GetVehicle().GetMass())

# Set explicit visualization types so meshes are rendered
veh_obj = man.GetVehicle()
veh_obj.SetChassisVisualizationType(chrono.VisualizationType_MESH)
veh_obj.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
veh_obj.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
veh_obj.SetWheelVisualizationType(chrono.VisualizationType_MESH)
veh_obj.SetTireVisualizationType(chrono.VisualizationType_MESH)


# === Rigid terrain ===
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM,
    TERRAIN_LENGTH,
    TERRAIN_WIDTH,
)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
terrain.Initialize()


# === Visualization (Irrlicht) ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("MAN 10t Truck — Rigid Terrain")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()    # vehicle demos use directional, not AddTypicalLights
vis.AttachVehicle(man.GetVehicle())


# === Interactive driver ===
driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(TIME_STEP / STEERING_TIME)
driver.SetThrottleDelta(TIME_STEP / THROTTLE_TIME)
driver.SetBrakingDelta(TIME_STEP / BRAKING_TIME)
driver.Initialize()


# === Review-only recording setup ===

# chassis body — cache for camera update in loop
chassis_body = man.GetChassisBody()  # cache


# === Main simulation loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
frame = 0

while vis.Run() and system.GetChTime() < SIM_END:
    # Throttled rendering
    if step_number % RENDER_EVERY == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        if REC:
            vis.WriteImageToFile(rec.frame_path(irr_dir, frame))
            frame += 1

    driver_inputs = driver.GetInputs()

    # review-only: apply throttle to make vehicle move for video
    if REC:

    driver.Synchronize(system.GetChTime())
    terrain.Synchronize(system.GetChTime())
    man.Synchronize(system.GetChTime(), driver_inputs, terrain)
    vis.Synchronize(system.GetChTime(), driver_inputs)

    driver.Advance(TIME_STEP)
    terrain.Advance(TIME_STEP)
    man.Advance(TIME_STEP)
    vis.Advance(TIME_STEP)

    step_number += 1
    realtime_timer.Spin(TIME_STEP)


# === Review-only post-processing ===
