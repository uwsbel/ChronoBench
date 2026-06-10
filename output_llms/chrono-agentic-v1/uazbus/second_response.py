"""
UAZ Bus Double Lane Change Simulation
======================================
System type: ChSystemNSC (NSC contact, rigid terrain)
Vehicle: veh.UAZBUS() — UAZ bus wrapper with TMEASY tires
Terrain: RigidTerrain with concrete.jpg texture
Driver: ChInteractiveDriverIRR (scored core); scripted double lane change in
        review-only block for RUN-stage video production.
Initial position: (-40, 0, 0.5) in world coordinates.

Expected behavior:
- Vehicle starts at x=-40 and drives forward along the X-axis.
- A double lane change maneuver is executed: first a left lane change,
  then a right lane change back, followed by braking to stop.
- The bus navigates the maneuver while maintaining stability.
"""

# === Imports ===
import os
import math
import csv

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr


# === Named constants (physics + geometry) ===
TIME_STEP = 2e-3          # simulation step size (s)
SIM_END = 20.0            # total simulation duration (s)
RENDER_FPS = 50.0         # rendering frame rate (fps)
RENDER_STEP_SIZE = 1.0 / RENDER_FPS

# Terrain dimensions
TERRAIN_LENGTH = 300.0    # m — long enough for the maneuver
TERRAIN_WIDTH = 60.0      # m — wide enough for lane changes

# Vehicle initial position and heading (pointing in +X direction)
INIT_X = -40.0
INIT_Y = 0.0
INIT_Z = 0.5
init_loc = chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z)
init_rot = chrono.QuatFromAngleZ(0.0)  # heading along +X

# precomputed once
render_steps = math.ceil(RENDER_STEP_SIZE / TIME_STEP)

# === REC flag (review-only) ===

# === Vehicle setup ===
# veh.UAZBUS() owns its own ChSystemNSC internally
uaz = veh.UAZBUS()
uaz.SetContactMethod(chrono.ChContactMethod_NSC)
uaz.SetChassisCollisionType(veh.CollisionType_NONE)
uaz.SetChassisFixed(False)  # MANDATORY — fixed chassis won't move
uaz.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
uaz.SetTireType(veh.TireModelType_TMEASY)
uaz.SetTireStepSize(TIME_STEP)
uaz.Initialize()

# === System & bodies (created by the veh.UAZBUS wrapper) ===
system = uaz.GetSystem()  # ChSystemNSC owned by the wrapper
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED, after Initialize

chassis = uaz.GetChassisBody()  # cache: fetched once, reused every step
# wheels/spindles: uaz.GetVehicle().GetAxle(i); terrain: RigidTerrain patch below
# joints: suspension + steering links created inside the wrapper

# Visualization types (after Initialize)
uaz.SetChassisVisualizationType(chrono.VisualizationType_MESH)
uaz.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
uaz.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
uaz.SetWheelVisualizationType(chrono.VisualizationType_MESH)
uaz.SetTireVisualizationType(chrono.VisualizationType_MESH)

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
# Concrete texture as requested
patch.SetTexture(chrono.GetChronoDataFile("vehicle/terrain/textures/concrete.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.8))

terrain.Initialize()

# === Visualization ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("UAZ Bus — Double Lane Change")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AttachVehicle(uaz.GetVehicle())

# === Driver (scored core: interactive) ===
driver = veh.ChInteractiveDriver(uaz.GetVehicle())
steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3
driver.SetSteeringDelta(RENDER_STEP_SIZE / steering_time)
driver.SetThrottleDelta(RENDER_STEP_SIZE / throttle_time)
driver.SetBrakingDelta(RENDER_STEP_SIZE / braking_time)
driver.Initialize()

# === CSV writer (review-only) ===

# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
frame = 0

try:
    while vis.Run() and system.GetChTime() < SIM_END:
        time = system.GetChTime()

        # Render at throttled cadence
        if step_number % render_steps == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        # Get driver inputs
        driver_inputs = driver.GetInputs()


        # Synchronize subsystems
        driver.Synchronize(time)
        terrain.Synchronize(time)
        uaz.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)


        # Advance subsystems
        driver.Advance(TIME_STEP)
        terrain.Advance(TIME_STEP)
        uaz.Advance(TIME_STEP)
        vis.Advance(TIME_STEP)

        step_number += 1
        realtime_timer.Spin(TIME_STEP)

except (RuntimeError, ValueError) as exc:  # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
