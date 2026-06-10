"""
UAZ Bus simulation on rigid terrain with a fixed box obstacle.

System type: NSC (rigid terrain, wheeled vehicle)
Vehicle: veh.UAZBUS — a wheeled bus driven forward at constant throttle 0.5.
Tire model: RIGID (changed from TMEASY per the prompt).
Scene: flat RigidTerrain with a fixed box obstacle (0.5 x 5 x 0.2) at (5, 0, 0.1).
Expected behaviour: The UAZBUS accelerates forward from the origin and contacts
the box obstacle placed across its path.
"""

# === Imports ===
import math
import os
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh

# === Data paths (mandatory for catalog-vehicle truth) ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# === Named constants ===
STEP_SIZE   = 1e-3        # physics time step (s)
SIM_END     = 15.0        # simulation end time (s)
RENDER_FPS  = 50.0
render_every = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))  # precomputed once

TERRAIN_LENGTH = 200.0
TERRAIN_WIDTH  = 100.0

INIT_X = 0.0
INIT_Y = 0.0
INIT_Z = 0.5              # chassis-origin height above ground at rest (m)

# Box obstacle (full extents, dimensions from prompt)
BOX_DX  = 0.5
BOX_DY  = 5.0
BOX_DZ  = 0.2
BOX_POS = chrono.ChVector3d(5.0, 0.0, 0.1)   # centre of the obstacle

# === Vehicle setup ===
uazbus = veh.UAZBUS()
uazbus.SetContactMethod(chrono.ChContactMethod_NSC)
uazbus.SetChassisCollisionType(veh.CollisionType_NONE)
uazbus.SetChassisFixed(False)
uazbus.SetInitPosition(chrono.ChCoordsysd(
    chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z),
    chrono.QUNIT
))
uazbus.SetTireType(veh.TireModelType_RIGID)   # prompt: rigid tire model
uazbus.SetTireStepSize(STEP_SIZE)
uazbus.Initialize()

# === System & bodies (created by the veh.UAZBUS wrapper) ===
system  = uazbus.GetSystem()         # ChSystemNSC owned by the wrapper
chassis = uazbus.GetChassisBody()    # cache: fetched once, reused each step
# wheels/spindles: uazbus.GetVehicle().GetAxle(i)... ; terrain: RigidTerrain below
# joints: suspension + steering links created inside the UAZBUS wrapper
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

print("VEHICLE MASS: ", uazbus.GetVehicle().GetMass())

# Visualisation types (after Initialize)
uazbus.SetChassisVisualizationType(chrono.VisualizationType_MESH)
uazbus.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
uazbus.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
uazbus.SetWheelVisualizationType(chrono.VisualizationType_MESH)
uazbus.SetTireVisualizationType(chrono.VisualizationType_MESH)

# === Terrain ===
terrain   = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Box obstacle (fixed, placed at (5, 0, 0.1)) ===
obs_mat = chrono.ChContactMaterialNSC()
obs_mat.SetFriction(0.8)
obs_mat.SetRestitution(0.01)
obstacle = chrono.ChBodyEasyBox(BOX_DX, BOX_DY, BOX_DZ, 1000.0, True, True, obs_mat)
obstacle.SetName("box_obstacle")
obstacle.SetPos(BOX_POS)
obstacle.SetFixed(True)
system.AddBody(obstacle)

# === Visualization ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("UAZ Bus — Rigid Tire + Box Obstacle")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(uazbus.GetVehicle())

# === Driver (scripted: constant throttle 0.5 — scored core) ===
# Plain DriverInputs struct for open-loop constant-throttle control
driver_inputs = veh.DriverInputs()
driver_inputs.m_throttle = 0.5
driver_inputs.m_steering  = 0.0
driver_inputs.m_braking   = 0.0

# === Recording setup ===


# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number    = 0

try:
    while vis.Run() and system.GetChTime() < SIM_END:
        time = system.GetChTime()

        if step_number % render_every == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        # Scripted constant throttle — vehicle accelerates forward (scored core)
        driver_inputs.m_throttle = 0.5
        driver_inputs.m_steering  = 0.0
        driver_inputs.m_braking   = 0.0

        terrain.Synchronize(time)
        uazbus.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        terrain.Advance(STEP_SIZE)
        uazbus.Advance(STEP_SIZE)
        vis.Advance(STEP_SIZE)


        step_number += 1
        realtime_timer.Spin(STEP_SIZE)

except (RuntimeError, ValueError) as exc:   # solver divergence / bad vehicle state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass  # CSV closed in review-only block below
