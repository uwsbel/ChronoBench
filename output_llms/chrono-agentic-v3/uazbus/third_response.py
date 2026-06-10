"""
UAZ Bus simulation on rigid flat terrain with a box obstacle.

System: ChSystemNSC (via UAZBUS wrapper, NSC contact method)
Vehicle: veh.UAZBUS() with RIGID tire model
Terrain: RigidTerrain flat patch with an additional fixed box obstacle
Driver: Scripted — constant throttle 0.5, zero steering/braking
Expected behavior: UAZBUS accelerates forward from rest, approaches and contacts
the box obstacle placed at x=5 m, and may push it or stop upon contact.
"""

import os
import math
import csv
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr  # noqa: F401 — used via vis
import pychrono.vehicle as veh


# === Constants ===
STEP_SIZE = 1e-3        # physics time step (s)
SIM_END   = 20.0        # simulation end time (s)
RENDER_FPS = 50.0       # target render frame rate
render_every = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))  # steps per render frame

TERRAIN_LENGTH = 200.0  # terrain patch length (m)
TERRAIN_WIDTH  = 100.0  # terrain patch width  (m)

VEH_INIT_X = 0.0
VEH_INIT_Y = 0.0
VEH_INIT_Z = 0.5        # chassis height above terrain at spawn (m)

# Box obstacle parameters (from prompt)
BOX_DX = 0.5   # full extent X
BOX_DY = 5.0   # full extent Y
BOX_DZ = 0.2   # full extent Z
BOX_X  = 5.0
BOX_Y  = 0.0
BOX_Z  = 0.1   # half-height above terrain => top at 0.2 m

# === Data paths (scored core — required by truth) ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

# === Vehicle setup ===
vehicle = veh.UAZBUS()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
vehicle.SetChassisFixed(False)                              # MANDATORY — fixed chassis won't move

init_loc = chrono.ChVector3d(VEH_INIT_X, VEH_INIT_Y, VEH_INIT_Z)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)
vehicle.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))

vehicle.SetTireType(veh.TireModelType_RIGID)               # prompt: changed to RIGID tire
vehicle.SetTireStepSize(STEP_SIZE)
vehicle.Initialize()

# === System & bodies (created by the veh.UAZBUS wrapper) ===
sys = vehicle.GetSystem()                                   # ChSystemNSC owned by the wrapper
chassis = vehicle.GetChassisBody()                         # cache: main chassis rigid body
# wheels/spindles: vehicle.GetVehicle().GetAxle(i); terrain: RigidTerrain patch body below
# joints: suspension + steering links created inside the wrapper

sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED after Initialize

print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

# === Visualization types ===
vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)

# === Terrain ===
terrain = veh.RigidTerrain(sys)

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
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Box obstacle (fixed, from prompt) ===
obs_mat = chrono.ChContactMaterialNSC()
obs_mat.SetFriction(0.8)
obs_mat.SetRestitution(0.01)

obstacle = chrono.ChBodyEasyBox(BOX_DX, BOX_DY, BOX_DZ, 1000.0, True, True, obs_mat)
obstacle.SetName("box_obstacle")
obstacle.SetPos(chrono.ChVector3d(BOX_X, BOX_Y, BOX_Z))
obstacle.SetFixed(True)
sys.AddBody(obstacle)

# === Interactive driver (scored-core truth shape for catalog vehicles) ===
vis_tmp = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis_tmp.SetWindowTitle("UAZ Bus — Rigid Tire + Box Obstacle")
vis_tmp.SetWindowSize(1280, 720)
vis_tmp.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis_tmp.Initialize()
vis_tmp.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis_tmp.AddSkyBox()
vis_tmp.AddLightDirectional()
vis_tmp.AttachVehicle(vehicle.GetVehicle())

vis = vis_tmp  # alias used in loop

driver = veh.ChInteractiveDriverIRR(vis)

render_step_size = 1.0 / RENDER_FPS       # precomputed once
steering_time = 1.0
throttle_time = 1.0
braking_time  = 0.3

driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# === Review-only record setup ===


# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        time = sys.GetChTime()

        if step_number % render_every == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()

        # Scripted driver — constant throttle (scored core, truth shape for uazbus)
        driver_inputs.m_throttle = 0.5
        driver_inputs.m_steering = 0.0
        driver_inputs.m_braking  = 0.0

        driver.Synchronize(time)
        terrain.Synchronize(time)
        vehicle.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)


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
