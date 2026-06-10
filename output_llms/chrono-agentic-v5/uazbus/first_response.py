"""UAZBUS wheeled-vehicle simulation on flat rigid terrain (PyChrono 9.0.0, Irrlicht).

Models the catalog UAZBUS van driving forward on a flat RigidTerrain patch with
defined friction and restitution. The vehicle is an NSC (non-smooth contact)
wrapper-owned system; a scripted driver applies a constant steering/throttle so
the bus accelerates forward in real time. Visualization is the vehicle-aware
Irrlicht chase-camera window. Expected behavior: the bus rolls forward along +X,
its wheels resting on (not through) the terrain, with stable real-time stepping.
"""

import math

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Parameters === geometry / physics constants (no bare literals downstream)
time_step = 1e-3                 # integration step (s)
sim_end = 12.0                   # bounded recording horizon (s)
render_fps = 50.0               # target render cadence (frames/s)

terrain_length = 200.0          # rigid patch X extent (m)
terrain_width = 100.0           # rigid patch Y extent (m)
terrain_friction = 0.8          # tire-ground friction coefficient
terrain_restitution = 0.01     # tire-ground bounciness

init_x = 0.0                     # spawn X (m)
init_y = 0.0                     # spawn Y (m)
init_z = 0.5                     # chassis-origin height above flat ground (m)
TIRE_RADIUS = 0.4               # nominal UAZBUS tire radius for footprint check (m)
TERRAIN_TOP_Z = 0.0             # flat patch top plane (m)
ZTOL = 0.1                       # allowed wheel-bottom clearance/overlap vs ground

throttle_cmd = 0.5              # scripted constant throttle (uazbus drive-forward)
steering_cmd = 0.0              # scripted steering (straight ahead)
braking_cmd = 0.0              # scripted braking

# === Data paths === locate bundled Chrono + vehicle assets (truth components)
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# === Vehicle === catalog UAZBUS wrapper owns its ChSystemNSC + all sub-bodies
init_loc = chrono.ChVector3d(init_x, init_y, init_z)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)

vehicle = veh.UAZBUS()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)   # NSC for rigid terrain
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
vehicle.SetChassisFixed(False)                          # MANDATORY — fixed chassis won't move
vehicle.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
vehicle.SetTireType(veh.TireModelType_TMEASY)           # rigid-terrain tire model
vehicle.SetTireStepSize(time_step)
vehicle.Initialize()

vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)

# === System & bodies (created by the veh.UAZBUS wrapper) ===
system = vehicle.GetSystem()                       # ChSystemNSC owned by the wrapper
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED for contact
chassis = vehicle.GetChassisBody()                 # cache: main chassis rigid body, reused below
# spindles: vehicle.GetVehicle().GetSpindlePos(axle, side); joints: suspension + steering inside wrapper
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())   # report total vehicle mass

# Footprint check — wheels must rest on (not through) the flat ground.
veh_obj = vehicle.GetVehicle()                     # cache: vehicle handle for spindle query
spindle_world = []
for axle in range(veh_obj.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        spindle_world.append(veh_obj.GetSpindlePos(axle, side))
wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS
assert wheel_bottom_z >= TERRAIN_TOP_Z - ZTOL, (
    f"vehicle sinks into ground: wheel bottom z={wheel_bottom_z:.3f} "
    f"vs ground top z={TERRAIN_TOP_Z:.3f}; raise init_z"
)

# === Terrain === flat rigid patch with defined friction + restitution
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(terrain_friction)
patch_mat.SetRestitution(terrain_restitution)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, terrain_length, terrain_width)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Visualization === vehicle-aware Irrlicht chase-camera window + sky + lights
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("UAZBUS on Rigid Terrain")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()                          # vehicle truths use a directional light
vis.AttachVehicle(vehicle.GetVehicle())

# === Driver === scripted constant inputs drive the bus straight forward
driver = veh.ChDataDriver(vehicle.GetVehicle(), veh.vector_Entry([
    veh.DataDriverEntry(0.0, steering_cmd, 0.0, 0.0),
    veh.DataDriverEntry(0.5, steering_cmd, 0.0, 0.0),
    veh.DataDriverEntry(0.7, steering_cmd, throttle_cmd, braking_cmd),
    veh.DataDriverEntry(sim_end, steering_cmd, throttle_cmd, braking_cmd),
]))
driver.Initialize()

# === Main loop === real-time Synchronize/Advance of driver, terrain, vehicle, vis
render_step_size = 1.0 / render_fps                 # precomputed once
render_steps = math.ceil(render_step_size / time_step)   # precomputed once


realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
frame = 0
try:
    while vis.Run() and system.GetChTime() < sim_end:
        time = system.GetChTime()

        if step_number % render_steps == 0:        # throttled rendering
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()


        driver.Synchronize(time)
        terrain.Synchronize(time)
        vehicle.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        driver.Advance(time_step)
        terrain.Advance(time_step)
        vehicle.Advance(time_step)
        vis.Advance(time_step)

        step_number += 1
        realtime_timer.Spin(time_step)             # spin so wall-clock matches sim time
except (RuntimeError, ValueError) as exc:          # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise

# === Post-processing === assemble the review video + flush logged physics
