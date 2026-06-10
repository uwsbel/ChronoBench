"""
UAZBUS vehicle simulation with rigid terrain.

System type: ChSystemNSC (NSC contact, created internally by the UAZBUS wrapper).
Main bodies: UAZBUS chassis + 4 wheel spindles + tire assemblies.
Terrain: RigidTerrain flat patch with defined friction and restitution.
Driver: ChInteractiveDriverIRR (real-time keyboard control via Irrlicht).
Expected behavior: UAZBUS drives on flat rigid terrain in real-time;
operator can steer, throttle, and brake via keyboard.
Visualization: Irrlicht chase-camera window with sky, lights, and terrain texture.
"""

import math
import os


import pychrono.core as chrono
import pychrono.vehicle as veh

# Anchor the bundled vehicle data subtree so mesh files resolve correctly
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

# === Named constants ===
STEP_SIZE = 1e-3          # simulation timestep (s)
SIM_END = 15.0            # simulation duration (s)
RENDER_FPS = 50.0         # target render / frame-capture rate (fps)

# Terrain parameters
TERRAIN_LENGTH = 200.0    # m
TERRAIN_WIDTH = 200.0     # m
TERRAIN_FRICTION = 0.8    # surface friction coefficient
TERRAIN_RESTITUTION = 0.01  # surface restitution coefficient

# UAZBUS initial position
INIT_X = 0.0
INIT_Y = 0.0
INIT_Z = 0.5              # spawn height above terrain (approx suspension ref height)

# Camera chase settings
CHASE_POINT = chrono.ChVector3d(0.0, 0.0, 1.75)
CHASE_DIST = 9.0
CHASE_HEIGHT = 0.5

# Interactive driver deltas
STEERING_TIME = 1.0       # s to full lock
THROTTLE_TIME = 1.0       # s to full throttle
BRAKING_TIME = 0.3        # s to full brake

# Precomputed rendering cadence (precomputed once, not inside loop)
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))

# === Vehicle setup ===
uazbus = veh.UAZBUS()
uazbus.SetContactMethod(chrono.ChContactMethod_NSC)  # NSC for rigid terrain
uazbus.SetChassisCollisionType(veh.CollisionType_NONE)
uazbus.SetChassisFixed(False)  # MANDATORY — fixed chassis never moves
uazbus.SetInitPosition(
    chrono.ChCoordsysd(
        chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z),
        chrono.QUNIT,
    )
)
uazbus.SetTireType(veh.TireModelType_TMEASY)
uazbus.SetTireStepSize(STEP_SIZE)
uazbus.Initialize()

# === System & bodies (created by the veh.UAZBUS wrapper) ===
system = uazbus.GetSystem()  # ChSystemNSC owned by the UAZBUS wrapper
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED — contact

chassis = uazbus.GetChassisBody()  # cache: main chassis rigid body; reused below

# Visualization types (must come after Initialize; VisualizationType_* lives in veh namespace)
uazbus.SetChassisVisualizationType(veh.VisualizationType_MESH)
uazbus.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
uazbus.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
uazbus.SetWheelVisualizationType(veh.VisualizationType_MESH)
uazbus.SetTireVisualizationType(veh.VisualizationType_MESH)

# === Terrain ===
terrain = veh.RigidTerrain(system)

patch_mat = chrono.ChContactMaterialNSC()  # NSC matches system contact method
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

# === Visualization — Irrlicht vehicle visual system ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("UAZBUS Rigid Terrain Demo")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(CHASE_POINT, CHASE_DIST, CHASE_HEIGHT)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AttachVehicle(uazbus.GetVehicle())

# === Driver — interactive keyboard control (ChInteractiveDriverIRR) ===
driver = veh.ChInteractiveDriverIRR(vis)  # takes the visual system, not the vehicle
driver.SetSteeringDelta(1.0 / (RENDER_FPS * STEERING_TIME))
driver.SetThrottleDelta(1.0 / (RENDER_FPS * THROTTLE_TIME))
driver.SetBrakingDelta(1.0 / (RENDER_FPS * BRAKING_TIME))
driver.Initialize()

# === Spindle footprint assert (validation — wheel bottoms must rest on terrain) ===
TIRE_RADIUS = 0.47  # UAZBUS approximate tire radius (m)
ZTOL = 0.10
veh_obj = uazbus.GetVehicle()  # cache: vehicle object, used in loop
spindle_world = []
for axle_idx in range(veh_obj.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        p = veh_obj.GetSpindlePos(axle_idx, side)
        spindle_world.append(p)

wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS
assert wheel_bottom_z >= -ZTOL, (
    f"vehicle sinks into terrain: wheel bottom z={wheel_bottom_z:.3f}; "
    f"raise INIT_Z by {-wheel_bottom_z:.3f} m"
)

# === Review-only recording setup ===


# === Real-time simulation timer ===
realtime_timer = chrono.ChRealtimeStepTimer()

# === Main loop ===
frame = 0
step_number = 0

try:
    while vis.Run():
        time = system.GetChTime()  # cache: fetched once per outer iteration

        # Throttled rendering — render at RENDER_FPS, step physics at 1/STEP_SIZE
        if step_number % RENDER_EVERY == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        # Synchronize subsystems
        driver_inputs = driver.GetInputs()


        driver.Synchronize(time)
        terrain.Synchronize(time)
        uazbus.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)


        # Advance subsystems
        driver.Advance(STEP_SIZE)
        terrain.Advance(STEP_SIZE)
        uazbus.Advance(STEP_SIZE)   # advances the wrapper-owned ChSystem
        vis.Advance(STEP_SIZE)

        step_number += 1
        realtime_timer.Spin(STEP_SIZE)  # throttle to real-time

        if time >= SIM_END:
            break

except (RuntimeError, ValueError) as exc:  # solver divergence / bad vehicle state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
