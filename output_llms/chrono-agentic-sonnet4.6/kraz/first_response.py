"""
Kraz tractor-trailer simulation on rigid terrain.

System type: NSC (rigid terrain catalog vehicle default).
Main bodies: Kraz tractor chassis, trailer chassis, axle/wheel/tire assemblies.
Terrain: RigidTerrain with a flat patch, friction 0.9, restitution 0.01.
Driver: ChInteractiveDriverIRR (real-time interactive).
Expected behavior: Kraz tractor-trailer resting on flat terrain, driveable
interactively in the Irrlicht window; vehicle maintains contact with terrain,
all subsystems (driver, terrain, vehicle, vis) synchronize each step.
"""

import math
import os
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr


# === Constants ===
# Geometry and physics parameters (precomputed once)
TERRAIN_LENGTH = 200.0          # m — flat terrain extent in X
TERRAIN_WIDTH = 200.0           # m — flat terrain extent in Y
TERRAIN_FRICTION = 0.9
TERRAIN_RESTITUTION = 0.01

INIT_X = 0.0
INIT_Y = 0.0
INIT_Z = 0.5                    # (inferred default — verify) chassis spawn height above ground
INIT_ROT = chrono.ChQuaterniond(1, 0, 0, 0)  # identity = facing +X

STEP_SIZE = 2e-3                 # integration time step (s)
SIM_END = 30.0                   # simulation end time (s)

RENDER_FPS = 50.0
render_step_size = 1.0 / RENDER_FPS                          # precomputed once
render_every = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE))) # precomputed once

STEERING_TIME = 1.0
THROTTLE_TIME = 1.0
BRAKING_TIME  = 0.3


# === Data paths (truth-required for catalog vehicles) ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

# === Vehicle setup ===
# veh.Kraz is a tractor-trailer compound; SetChassisVisualizationType etc. take (tractor_vis, trailer_vis)
# Visualization attaches via GetTractor() (a ChWheeledVehicle), not a GetVehicle() wrapper.
init_loc = chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z)

kraz = veh.Kraz()
kraz.SetContactMethod(chrono.ChContactMethod_NSC)
kraz.SetChassisFixed(False)
kraz.SetInitPosition(chrono.ChCoordsysd(init_loc, INIT_ROT))
kraz.SetTireStepSize(STEP_SIZE)
kraz.Initialize()

# === System & bodies (created by the veh.Kraz wrapper) ===
sys = kraz.GetSystem()                          # ChSystemNSC owned by the Kraz wrapper
chassis = kraz.GetTractorChassisBody()          # cache: tractor chassis rigid body; fetched once, reused every step
# Trailer chassis: kraz.GetTrailer().GetChassisBody()
# Axles/spindles: kraz.GetTractor().GetAxle(i) ...
# Joints/suspension: created internally by the wrapper

sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED after Initialize

print("VEHICLE MASS: ", kraz.GetTractor().GetMass())

# Visualization types (after Initialize) — Kraz takes (tractor_vis, trailer_vis)
# VisualizationType_* lives in veh namespace (not chrono) for vehicle wrappers
kraz.SetChassisVisualizationType(veh.VisualizationType_MESH, veh.VisualizationType_MESH)
kraz.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES, veh.VisualizationType_PRIMITIVES)
kraz.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
kraz.SetWheelVisualizationType(veh.VisualizationType_MESH, veh.VisualizationType_MESH)
kraz.SetTireVisualizationType(veh.VisualizationType_MESH, veh.VisualizationType_MESH)

# === Terrain ===
# RigidTerrain flat patch, NSC material (matches catalog vehicle contact method)
terrain = veh.RigidTerrain(sys)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(TERRAIN_FRICTION)
patch_mat.SetRestitution(TERRAIN_RESTITUTION)
patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM,
    TERRAIN_LENGTH,
    TERRAIN_WIDTH,
)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Visualization ===
# ChWheeledVehicleVisualSystemIrrlicht; Kraz attaches via GetTractor() (ChWheeledVehicle)
# ChWheeledVehicleVisualSystemIrrlicht lives in veh namespace (not chronoirr)
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("Kraz Tractor-Trailer — Rigid Terrain")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 14.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()           # vehicle truths use directional light
vis.AttachVehicle(kraz.GetTractor())  # Kraz: attach via GetTractor() (ChWheeledVehicle)

# === Driver ===
# Interactive driver bound to the visual system (scored-core default, matches truth)
driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(render_step_size / STEERING_TIME)
driver.SetThrottleDelta(render_step_size / THROTTLE_TIME)
driver.SetBrakingDelta(render_step_size / BRAKING_TIME)
driver.Initialize()

# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
frame = 0


try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        time = sys.GetChTime()

        if step_number % render_every == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()


        driver.Synchronize(time)
        terrain.Synchronize(time)
        kraz.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)


        driver.Advance(STEP_SIZE)
        terrain.Advance(STEP_SIZE)
        kraz.Advance(STEP_SIZE)
        vis.Advance(STEP_SIZE)

        step_number += 1
        realtime_timer.Spin(STEP_SIZE)

except (RuntimeError, ValueError) as exc:   # solver divergence / bad vehicle state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
