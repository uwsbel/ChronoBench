"""
Gator vehicle simulation on rigid terrain — PyChrono 9.0.x, Irrlicht renderer.

System type: NSC (rigid terrain default for Gator).
Main bodies: Gator chassis, wheels/tires, rigid terrain patch.
Expected behavior: Gator drives forward on flat terrain under interactive keyboard
    control. Chassis uses primitive box collision shapes. Visualization is set to
    primitives for chassis/suspension/steering/wheels, mesh for tires. Driver
    response times are increased so control inputs ramp up slowly.
"""

import os
import math
import csv

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


# === Constants ===
STEP_SIZE = 5e-4        # physics time step (s)
SIM_END = 20.0          # simulation end time (s)
RENDER_FPS = 50.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))  # precomputed once

TERRAIN_LENGTH = 200.0
TERRAIN_WIDTH = 100.0

INIT_LOC = chrono.ChVector3d(0.0, 0.0, 0.5)
INIT_ROT = chrono.ChQuaterniond(1, 0, 0, 0)

# Driver response times (seconds to go 0 → max) — INCREASED for less responsiveness
STEERING_TIME = 2.0   # slower steering response
THROTTLE_TIME = 2.0   # slower throttle response
BRAKING_TIME  = 1.0   # slower braking response

# === Data paths ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# === Vehicle ===
gator = veh.Gator()
gator.SetContactMethod(chrono.ChContactMethod_NSC)
gator.SetChassisCollisionType(veh.CollisionType_NONE)  # manual primitives added below
gator.SetChassisFixed(False)
gator.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
gator.SetTireType(veh.TireModelType_TMEASY)
gator.SetTireStepSize(STEP_SIZE)
gator.Initialize()

print("VEHICLE MASS: ", gator.GetVehicle().GetMass())

# === System & bodies (created by the veh.Gator wrapper) ===
system = gator.GetSystem()   # ChSystemNSC owned by the wrapper
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

chassis = gator.GetChassisBody()  # cache: fetched once, reused

# Add primitive box collision shapes to chassis for chassis-collision
# (prompt: "keep the collision simple with primitive shapes")
cmat = chrono.ChContactMaterialNSC()
cmat.SetFriction(0.7)
cmat.SetRestitution(0.02)
chassis.AddCollisionShape(
    chrono.ChCollisionShapeBox(cmat, 2.8, 1.4, 0.4),
    chrono.ChFramed(chrono.ChVector3d(0.0, 0.0, 0.3), chrono.QUNIT),
)
chassis.EnableCollision(True)
system.GetCollisionSystem().BindAll()

# === Visualization types — PRIMITIVES (prompt: simplify from mesh to primitives) ===
gator.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
gator.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
gator.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
gator.SetWheelVisualizationType(veh.VisualizationType_PRIMITIVES)
gator.SetTireVisualizationType(veh.VisualizationType_PRIMITIVES)

# === Terrain ===
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
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Visualization (Irrlicht) ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("Gator — Primitives Visualization")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(gator.GetVehicle())

# === Driver — interactive IRR with INCREASED response times ===
render_step_size = 1.0 / RENDER_FPS  # precomputed once

driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(render_step_size / STEERING_TIME)  # slower response
driver.SetThrottleDelta(render_step_size / THROTTLE_TIME)
driver.SetBrakingDelta(render_step_size / BRAKING_TIME)
driver.Initialize()

# === Recording setup (review-only) ===

# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
frame = 0

try:
    while vis.Run() and system.GetChTime() < SIM_END:
        time = system.GetChTime()

        if step_number % RENDER_EVERY == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()

        # review-only block: scripted override to make vehicle move for RUN video

        driver.Synchronize(time)
        terrain.Synchronize(time)
        gator.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)


        driver.Advance(STEP_SIZE)
        terrain.Advance(STEP_SIZE)
        gator.Advance(STEP_SIZE)
        vis.Advance(STEP_SIZE)

        step_number += 1
        realtime_timer.Spin(STEP_SIZE)

except (RuntimeError, ValueError) as exc:   # solver divergence / bad vehicle state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
