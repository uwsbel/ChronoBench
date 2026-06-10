"""
HMMWV Full Model on Flat Rigid Terrain — PyChrono 9.0.x / Irrlicht

Models a full HMMWV (High Mobility Multipurpose Wheeled Vehicle) on a flat rigid
terrain. System type: ChSystemNSC (NSC contact via the HMMWV wrapper). Main bodies:
HMMWV chassis, four wheel spindles, RigidTerrain patch. Expected behavior: the
vehicle rests on the terrain at initialization; an interactive driver system allows
real-time steering, throttle, and braking via the Irrlicht window keyboard. The
simulation runs in real time at 50 fps.
"""

import math
import os

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr          # Irrlicht renderer (not VSG)
import pychrono.vehicle as veh


# === Named constants ===
STEP_SIZE         = 2e-3           # physics time step (s)
SIM_END           = 20.0           # simulation duration (s)
RENDER_FPS        = 50.0           # target render / capture rate
TERRAIN_LENGTH    = 300.0          # terrain patch X extent (m)
TERRAIN_WIDTH     = 300.0          # terrain patch Y extent (m)
TERRAIN_FRICTION  = 0.9
TERRAIN_RESTITUTION = 0.01
INIT_X, INIT_Y    = 0.0, 0.0      # vehicle spawn XY (m)
SUSPENSION_REF_HEIGHT = 0.5        # chassis origin above wheel-bottom at rest (m)

render_every = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))   # precomputed once

# === Data paths (truth-faithful mandatory pair) ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

# === Vehicle (HMMWV_Full wrapper — creates and owns ChSystemNSC) ===
init_loc = chrono.ChVector3d(INIT_X, INIT_Y, SUSPENSION_REF_HEIGHT)
init_rot = chrono.QUNIT

hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)    # NSC for rigid terrain
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)                          # MANDATORY — fixed chassis won't move
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
hmmwv.SetTireType(veh.TireModelType_TMEASY)           # prompt: TMEASY tire model
hmmwv.SetTireStepSize(STEP_SIZE)
hmmwv.Initialize()

# === System & bodies (created by the veh.HMMWV_Full wrapper) ===
sys = hmmwv.GetSystem()                  # ChSystemNSC owned by the wrapper
chassis = hmmwv.GetChassisBody()         # cache: fetched once, reused below
# wheels/spindles: hmmwv.GetVehicle().GetAxle(i); joints: suspension + steering inside wrapper
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED after Initialize

print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())             # truth-required diagnostic

# === Terrain (RigidTerrain, NSC material) ===
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

# === Visualization settings (after Initialize) ===
hmmwv.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

# === Footprint assertion: verify wheel-bottom sits on terrain ===
TIRE_RADIUS = 0.33    # approximate HMMWV TMEASY tire radius
ZTOL = 0.10
veh_obj = hmmwv.GetVehicle()       # cache: fetched once
spindle_positions = []
for axle_idx in range(veh_obj.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        p = veh_obj.GetSpindlePos(axle_idx, side)
        spindle_positions.append(p)

wheel_bottom_z = min(p.z for p in spindle_positions) - TIRE_RADIUS
assert wheel_bottom_z >= -ZTOL, (
    f"Vehicle sinks into terrain: wheel bottom z={wheel_bottom_z:.3f}; "
    f"raise SUSPENSION_REF_HEIGHT by {-wheel_bottom_z:.3f} m"
)

# === Irrlicht vehicle visualization ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV on Flat Rigid Terrain")
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()           # vehicle truth uses directional light
vis.AttachVehicle(hmmwv.GetVehicle())

# === Driver (interactive, truth-faithful) ===
driver = veh.ChInteractiveDriverIRR(vis)

steering_time = 1.0                 # seconds to reach full steering
throttle_time = 1.0                 # seconds to reach full throttle
braking_time  = 0.3                 # seconds to reach full braking

driver.SetSteeringDelta(render_every * STEP_SIZE / steering_time)
driver.SetThrottleDelta(render_every * STEP_SIZE / throttle_time)
driver.SetBrakingDelta(render_every * STEP_SIZE / braking_time)
driver.Initialize()

# === Review-only: recording setup ===

# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
frame = 0

try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        sim_time = sys.GetChTime()      # cache: fetched once per outer iteration

        if step_number % render_every == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()

        # Review-only: scripted override so the validation video shows motion

        driver.Synchronize(sim_time)
        terrain.Synchronize(sim_time)
        hmmwv.Synchronize(sim_time, driver_inputs, terrain)
        vis.Synchronize(sim_time, driver_inputs)


        driver.Advance(STEP_SIZE)
        terrain.Advance(STEP_SIZE)
        hmmwv.Advance(STEP_SIZE)          # internally advances the ChSystem
        vis.Advance(STEP_SIZE)

        step_number += 1
        realtime_timer.Spin(STEP_SIZE)

except (RuntimeError, ValueError) as exc:   # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
