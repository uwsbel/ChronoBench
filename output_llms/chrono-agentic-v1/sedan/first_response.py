"""
BMW E90 Sedan on Rigid Terrain — Interactive Driver Demo

This simulation showcases the dynamics of a BMW E90 Sedan driving on a rigid
terrain patch. The vehicle uses the TMEASY tire model for realistic tire-road
interaction. An interactive driver (ChInteractiveDriverIRR) allows real-time
control of steering, throttle, and braking from the keyboard.

System type: ChSystemNSC (NSC contact via the BMW_E90 wrapper)
Main bodies: BMW E90 chassis + 4 wheel spindles (created by the veh.BMW_E90 wrapper),
             RigidTerrain patch body.
Expected behavior: The sedan rests on a flat rigid terrain and responds to
                   keyboard inputs for steering, throttle, and braking. A chase
                   camera follows the vehicle. A skybox, directional lights, and
                   a terrain texture are visible in the Irrlicht window.
"""

# === Imports ===
import math

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Named constants ===
TIME_STEP = 1e-3            # physics integration step (s)
SIM_END   = 30.0            # simulation end time (s); long enough for a driving demo

TERRAIN_LENGTH = 200.0      # m (X direction)
TERRAIN_WIDTH  = 200.0      # m (Y direction)
FRICTION       = 0.9
RESTITUTION    = 0.01

RENDER_FPS     = 50.0
RENDER_EVERY   = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # physics steps per frame; precomputed once

STEERING_TIME  = 1.0        # s to reach full lock
THROTTLE_TIME  = 1.0        # s to reach full throttle
BRAKING_TIME   = 0.3        # s to reach full brake

# BMW E90 suspension reference height (chassis-origin above wheel-bottom at rest)
SUSPENSION_REF_HEIGHT = 0.5  # m — confirmed from sedan demo geometry
TERRAIN_Z             = 0.0  # flat terrain top surface

init_z   = TERRAIN_Z + SUSPENSION_REF_HEIGHT
init_pos = chrono.ChVector3d(0.0, 0.0, init_z)
init_rot = chrono.QUNIT

# === Vehicle setup (BMW E90 wrapper — owns its ChSystem) ===
sedan = veh.BMW_E90()
sedan.SetContactMethod(chrono.ChContactMethod_NSC)
sedan.SetChassisCollisionType(veh.CollisionType_NONE)
sedan.SetChassisFixed(False)                               # MANDATORY — fixed chassis won't move
sedan.SetInitPosition(chrono.ChCoordsysd(init_pos, init_rot))
sedan.SetTireType(veh.TireModelType_TMEASY)                # prompt: TMEASY tire model
sedan.SetTireStepSize(TIME_STEP)
sedan.Initialize()

# === System & bodies (created by the veh.BMW_E90 wrapper) ===
system  = sedan.GetSystem()                # ChSystemNSC owned by the wrapper
chassis = sedan.GetChassisBody()           # main chassis rigid body  # cache: fetched once, reused
# wheels/spindles: sedan.GetVehicle().GetAxle(i)... ; terrain: RigidTerrain patch below
# joints: suspension + steering links created inside the wrapper

system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED after Initialize

# Visualization types (after Initialize) — VisualizationType_* lives in veh namespace here
# Use PRIMITIVES throughout: BMW E90 mesh files are not bundled in this 9.0.x data install
sedan.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
sedan.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
sedan.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
sedan.SetWheelVisualizationType(veh.VisualizationType_PRIMITIVES)
sedan.SetTireVisualizationType(veh.VisualizationType_PRIMITIVES)

# Assert wheel footprint after Initialize (flat terrain — check wheel-bottom Z only)
TIRE_RADIUS = 0.33          # approximate BMW E90 tire radius (m)
ZTOL        = 0.10          # allowed clearance/overlap vs terrain surface
veh_obj     = sedan.GetVehicle()           # cache: fetched once
spindle_zs  = []
for axle_idx in range(veh_obj.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        p = veh_obj.GetSpindlePos(axle_idx, side)
        spindle_zs.append(p.z)
wheel_bottom_z = min(spindle_zs) - TIRE_RADIUS
assert wheel_bottom_z >= TERRAIN_Z - ZTOL, (
    f"vehicle sinks into terrain: wheel_bottom_z={wheel_bottom_z:.3f} "
    f"vs terrain z={TERRAIN_Z:.3f}; raise SUSPENSION_REF_HEIGHT by "
    f"{TERRAIN_Z - wheel_bottom_z:.3f} m"
)

# === Terrain ===
terrain   = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(FRICTION)
patch_mat.SetRestitution(RESTITUTION)

patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM,
    TERRAIN_LENGTH,
    TERRAIN_WIDTH,
)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Irrlicht visualization (vehicle-specific window with chase camera) ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("BMW E90 Sedan — TMEASY Tires — Interactive Driver")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AttachVehicle(veh_obj)

# === Interactive driver (scored-core default for real-time catalog vehicle demos) ===
driver = veh.ChInteractiveDriverIRR(vis)   # takes the visual system, not the vehicle
render_step_size = 1.0 / RENDER_FPS        # precomputed once
driver.SetSteeringDelta(render_step_size / STEERING_TIME)
driver.SetThrottleDelta(render_step_size / THROTTLE_TIME)
driver.SetBrakingDelta(render_step_size / BRAKING_TIME)
driver.Initialize()

# === Review-only recording setup ===


# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
frame          = 0
step_number    = 0

try:
    while vis.Run():
        time = system.GetChTime()
        if time >= SIM_END:
            break

        # Throttled rendering
        if step_number % RENDER_EVERY == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()


        # Synchronize subsystems (driver → terrain → vehicle → vis)
        driver.Synchronize(time)
        terrain.Synchronize(time)
        sedan.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)


        # Advance subsystems
        driver.Advance(TIME_STEP)
        terrain.Advance(TIME_STEP)
        sedan.Advance(TIME_STEP)    # steps the wrapper-owned ChSystem
        vis.Advance(TIME_STEP)

        step_number += 1
        realtime_timer.Spin(TIME_STEP)

except (RuntimeError, ValueError) as exc:   # solver divergence / bad vehicle state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass  # nothing to close in the scored core
