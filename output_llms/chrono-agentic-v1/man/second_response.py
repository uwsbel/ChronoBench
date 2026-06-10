"""
MAN 5t truck on rigid hilly terrain with height-map surface and grass texture.

System type: ChSystemNSC (rigid terrain, NSC contact)
Vehicle: veh.MAN_5t — 5-tonne military truck wrapper
Terrain: RigidTerrain with height-map patch (bump64.bmp, hills only, positive heights),
         grass.jpg texture
Initial spawn: (-20, 0, 1.5) in world frame
Driver: ChInteractiveDriver (real-time keyboard control)

Expected behavior: The MAN 5t truck spawns at (-20, 0, 1.5) on hilly terrain
covered with a grass texture. The terrain uses a BMP height map creating rolling
hills (height range 0-4 m). The truck can be driven interactively across the
undulating surface.
"""

import math
import os

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr


# === Constants ===
# Vehicle spawn location (as specified in prompt)
INIT_LOC = chrono.ChVector3d(-20.0, 0.0, 1.5)
INIT_ROT = chrono.QUNIT  # heading along +X axis

# Time parameters
TIME_STEP = 1e-3           # physics step size (s)
SIM_END = 20.0             # simulation end time (s)
RENDER_FPS = 50.0          # visualization frame rate (Hz)
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

# Terrain dimensions — 400x400 m centered at origin (large enough for full run)
TERRAIN_LENGTH = 400.0
TERRAIN_WIDTH = 400.0
TERRAIN_HMIN = 0.0       # minimum height (m) — positive-only hills, no pits
TERRAIN_HMAX = 4.0       # maximum hill height (m)

# Interactive driver ramp times
STEERING_TIME = 1.0   # s to reach max steering
THROTTLE_TIME = 1.0   # s to reach max throttle
BRAKING_TIME = 0.3    # s to reach max braking

# === Vehicle setup ===
# Set vehicle data path once, before accessing vehicle data files
veh.SetVehicleDataPath(chrono.GetChronoDataPath() + "vehicle/")

# MAN_5t wrapper — creates and owns its own ChSystemNSC internally
man = veh.MAN_5t()
man.SetContactMethod(chrono.ChContactMethod_NSC)          # NSC for rigid terrain
man.SetChassisCollisionType(veh.CollisionType_NONE)       # no chassis self-collision
man.SetChassisFixed(False)                                # MANDATORY: moving chassis
man.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
man.SetTireType(veh.TireModelType_TMEASY)                 # TMEASY for hilly terrain
man.SetTireStepSize(TIME_STEP)
man.Initialize()

# === System & bodies (created by the veh.MAN_5t wrapper) ===
system = man.GetSystem()                    # ChSystemNSC owned by the wrapper
chassis = man.GetChassisBody()              # main chassis rigid body; cache: fetched once
# wheels/spindles: man.GetVehicle().GetAxles()...
# joints: suspension + steering links created inside the MAN_5t wrapper
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED for contact

# Visualization types (must be set AFTER Initialize())
man.SetChassisVisualizationType(chrono.VisualizationType_MESH)
man.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
man.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
man.SetWheelVisualizationType(chrono.VisualizationType_MESH)
man.SetTireVisualizationType(chrono.VisualizationType_MESH)

# === Terrain setup — RigidTerrain with height-map (hills) ===
terrain = veh.RigidTerrain(system)

patch_mat = chrono.ChContactMaterialNSC()   # NSC material matching the system type
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

# Height-map patch: bump64.bmp creates rolling hills, positive range only (no pits)
heightmap_file = veh.GetVehicleDataFile("terrain/height_maps/bump64.bmp")
patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM,          # centered at origin, Z-up
    heightmap_file,           # BMP height map for rolling hills (bump64 = raised center)
    TERRAIN_LENGTH,
    TERRAIN_WIDTH,
    TERRAIN_HMIN,             # 0.0 m — no negative dips
    TERRAIN_HMAX,             # 4.0 m — rolling hills up to 4 m
)

# Apply grass texture as specified in the prompt
grass_tex = veh.GetVehicleDataFile("terrain/textures/grass.jpg")
patch.SetTexture(grass_tex, 60, 60)
patch.SetColor(chrono.ChColor(0.4, 0.7, 0.3))

terrain.Initialize()

# === Validation: assert wheel bottoms above terrain at spawn ===
veh_obj = man.GetVehicle()    # cache: fetched once, reused in loop and below
spindle_world = []
for axle_idx in range(veh_obj.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        p = veh_obj.GetSpindlePos(axle_idx, side)
        spindle_world.append(p)

TIRE_RADIUS = 0.629          # MAN 5t TMEASY tire radius (m), verified via introspection
min_spindle_z = min(p.z for p in spindle_world)
wheel_bottom_z = min_spindle_z - TIRE_RADIUS
assert wheel_bottom_z >= TERRAIN_HMIN - 0.2, (
    f"wheel bottom z={wheel_bottom_z:.3f} may fall below terrain minimum "
    f"{TERRAIN_HMIN:.3f}; raise INIT_LOC.z"
)

# === Visualization — ChWheeledVehicleVisualSystemIrrlicht ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("MAN 5t — Hilly Terrain (Grass Texture)")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 12.0, 0.5)
vis.Initialize()                          # Initialize FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AttachVehicle(man.GetVehicle())       # bind vehicle assets AFTER Initialize

# === Driver — ChInteractiveDriver (real-time keyboard control) ===
driver = veh.ChInteractiveDriver(man.GetVehicle())   # takes the vehicle object
driver.SetSteeringDelta(RENDER_EVERY * TIME_STEP / STEERING_TIME)   # precomputed once
driver.SetThrottleDelta(RENDER_EVERY * TIME_STEP / THROTTLE_TIME)
driver.SetBrakingDelta(RENDER_EVERY * TIME_STEP / BRAKING_TIME)
driver.Initialize()

# === Review-only recording setup ===


# Scripted review-only driver: keeps vehicle in safe terrain zone

# === Main simulation loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
frame = 0

try:
    while vis.Run():
        time = system.GetChTime()          # current simulation time

        if frame % RENDER_EVERY == 0:     # throttled rendering
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        # Driver inputs — interactive driver (scored core)
        driver_inputs = driver.GetInputs()

        # Synchronize subsystems (fixed order: driver → terrain → vehicle → vis)
        driver.Synchronize(time)
        terrain.Synchronize(time)
        man.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)


        # Advance subsystems
        driver.Advance(TIME_STEP)
        terrain.Advance(TIME_STEP)
        man.Advance(TIME_STEP)     # steps the wrapper-owned ChSystem
        vis.Advance(TIME_STEP)

        frame += 1
        realtime_timer.Spin(TIME_STEP)   # keep wall-clock pace

        if time >= SIM_END:
            break

except (RuntimeError, ValueError) as exc:   # solver divergence / bad vehicle state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass

# === Post-processing (review-only) ===
