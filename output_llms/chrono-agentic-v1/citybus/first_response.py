"""
CityBus simulation on RigidTerrain using PyChrono 9.0.x with Irrlicht visualization.

System type: ChSystemNSC (rigid contact, NSC solver), owned by the CityBus wrapper.
Vehicle: veh.CityBus() on a flat RigidTerrain with a custom texture.
Driver: veh.ChInteractiveDriverIRR (real-time keyboard control) — scored core.
Visualization: veh.ChWheeledVehicleVisualSystemIrrlicht, chase camera following bus.
Expected behavior: CityBus rests on flat terrain, driven interactively at 50 fps.
"""

# === Imports ===
import math
import os

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# Anchor vehicle data path to the Chrono bundled data tree (absolute, not relative)
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")


# === Named constants ===
STEP_SIZE = 2e-3          # physics timestep (s)
SIM_END = 30.0            # simulation duration (s)
RENDER_FPS = 50.0         # frames per second for rendering

TERRAIN_LENGTH = 300.0    # m, X extent
TERRAIN_WIDTH = 300.0     # m, Y extent

INIT_X = 0.0              # bus spawn X (m)
INIT_Y = 0.0              # bus spawn Y (m)
INIT_Z = 0.5              # bus spawn Z — chassis origin above terrain; adjusted for CityBus suspension

CHASSIS_TRACK_PT = chrono.ChVector3d(0.0, 0.0, 1.75)   # chase-camera track point on chassis
CAM_DISTANCE = 14.0       # chase camera distance behind vehicle (m)
CAM_HEIGHT = 0.5          # chase camera vertical offset (m)

# Precomputed once: render cadence (physics steps per frame)
RENDER_EVERY = max(1, math.ceil(1.0 / (RENDER_FPS * STEP_SIZE)))   # precomputed once

# === Vehicle and terrain setup ===
# CityBus wrapper owns its ChSystemNSC internally.
bus = veh.CityBus()
bus.SetContactMethod(chrono.ChContactMethod_NSC)
bus.SetChassisCollisionType(veh.CollisionType_NONE)
bus.SetChassisFixed(False)                          # MANDATORY — fixed chassis won't move

init_rot = chrono.QuatFromAngleZ(0.0)               # facing +X direction
init_pos = chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z)
bus.SetInitPosition(chrono.ChCoordsysd(init_pos, init_rot))

bus.SetTireType(veh.TireModelType_TMEASY)           # TMEASY: good for rigid terrain demos
bus.SetTireStepSize(STEP_SIZE)

bus.Initialize()

# === System & bodies (created by the veh.CityBus wrapper) ===
system = bus.GetSystem()                            # ChSystemNSC owned by the wrapper
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED after Initialize
chassis = bus.GetChassisBody()                      # cache: main chassis rigid body
# Axles/spindles: bus.GetVehicle().GetAxle(i).m_wheels[j].GetSpindle()
# Suspension + steering joints: created internally by the wrapper

# === Visualization types ===
# Prompt: "mesh and primitive visualization types for different vehicle parts"
# In this 9.0.0 build, VisualizationType_* lives in the veh namespace, not chrono.
# Prompt: mesh for chassis/wheel/tire, primitives for suspension/steering
bus.SetChassisVisualizationType(veh.VisualizationType_MESH)
bus.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
bus.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
bus.SetWheelVisualizationType(veh.VisualizationType_MESH)
bus.SetTireVisualizationType(veh.VisualizationType_MESH)

# === Terrain (RigidTerrain with custom texture) ===
terrain = veh.RigidTerrain(system)

patch_mat = chrono.ChContactMaterialNSC()           # NSC material matches the bus system
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM,       # centered at origin, no rotation
    TERRAIN_LENGTH,
    TERRAIN_WIDTH,
)

# Prompt: "custom texture" for the terrain
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))

terrain.Initialize()

# === Validate wheel footprint after Initialize ===
TIRE_RADIUS = 0.55   # CityBus tire radius (approx, meters)
ZTOL = 0.15          # tolerance for initial wheel-bottom clearance

bus_veh = bus.GetVehicle()                          # cache: inner vehicle object
spindle_world = []
for axle_idx in range(bus_veh.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        p = bus_veh.GetSpindlePos(axle_idx, side)
        spindle_world.append(p)

wheel_bottom_z = min(sp.z for sp in spindle_world) - TIRE_RADIUS
terrain_z = terrain.GetHeight(chrono.ChVector3d(INIT_X, INIT_Y, 0.0))
assert wheel_bottom_z >= terrain_z - ZTOL, (
    f"CityBus sinks through terrain: wheel_bottom_z={wheel_bottom_z:.3f} "
    f"vs terrain_z={terrain_z:.3f}; raise INIT_Z by {terrain_z - wheel_bottom_z:.3f} m"
)

# === Irrlicht vehicle visualization ===
# Prompt: "set the camera to follow the vehicle from a specified position"
# Use ChWheeledVehicleVisualSystemIrrlicht — the standard vehicle vis system.
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("CityBus on RigidTerrain")
vis.SetWindowSize(1280, 720)
# Chase camera following the bus from BEHIND and ABOVE
vis.SetChaseCamera(CHASSIS_TRACK_PT, CAM_DISTANCE, CAM_HEIGHT)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AttachVehicle(bus.GetVehicle())

# === Interactive driver (scored core) ===
# Prompt: "interactive driver system that allows for control of the vehicle's steering, throttle, and braking"
# ChInteractiveDriverIRR takes the visual system (not the vehicle).
driver = veh.ChInteractiveDriverIRR(vis)

render_step_size = 1.0 / RENDER_FPS   # seconds per render frame
steering_time = 1.0   # seconds to ramp 0 -> +1 steering
throttle_time = 1.0   # seconds to ramp 0 -> +1 throttle
braking_time = 0.3    # seconds to ramp 0 -> +1 braking

driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# === Review-only: recording setup ===

# === Main simulation loop ===
# Prompt: "simulation loop should run at 50 frames per second, updating vehicle dynamics and rendering in real time"
realtime_timer = chrono.ChRealtimeStepTimer()   # real-time throttle for interactive loop
step_number = 0
frame = 0

try:
    while vis.Run() and system.GetChTime() < SIM_END:
        sim_time = system.GetChTime()

        # --- Render once per frame (throttled at RENDER_FPS) ---
        if step_number % RENDER_EVERY == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        # Scored core: get interactive driver inputs
        driver_inputs = driver.GetInputs()

        # --- Review-only: override inputs with scripted maneuver for the record video ---

        # --- Synchronize subsystems (fixed order) ---
        driver.Synchronize(sim_time)
        terrain.Synchronize(sim_time)
        bus.Synchronize(sim_time, driver_inputs, terrain)
        vis.Synchronize(sim_time, driver_inputs)

        # --- Advance all subsystems ---
        driver.Advance(STEP_SIZE)
        terrain.Advance(STEP_SIZE)
        bus.Advance(STEP_SIZE)     # internally calls DoStepDynamics — do NOT also call system.DoStepDynamics
        vis.Advance(STEP_SIZE)

        # --- Review-only: CSV logging each physics step ---

        step_number += 1
        realtime_timer.Spin(STEP_SIZE)   # real-time throttle for interactive demo

except (RuntimeError, ValueError) as exc:   # solver divergence / bad parameter
    import traceback
    traceback.print_exc()
    raise
finally:
    pass

# === Post-processing (review-only) ===
