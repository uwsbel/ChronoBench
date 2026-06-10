"""MAN 10t truck driving on flat rigid terrain (PyChrono / NSC).

Models a MAN 10t catalog wheeled vehicle on a flat RigidTerrain patch using a
TMEASY tire model and NSC contact. The vehicle is rendered with the wheeled-vehicle
Irrlicht visual system: a chase camera, directional lighting, a sky box, a logo,
and a textured terrain patch. Control is via an interactive (keyboard) driver for
steering, throttle, and braking. Expected behavior: the truck rests on the terrain
at spawn (wheels on the surface, not through it) and is driven in real time.
"""

import math

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Parameters === geometry / physics constants and derived spawn pose
time_step = 1e-3                 # integration step (s)
sim_end = 12.0                   # bounded recording run length (s)
render_step_size = 1.0 / 50.0    # render cadence (s)

terrain_length = 200.0           # terrain X extent (m)
terrain_width = 200.0            # terrain Y extent (m)
terrain_top_z = 0.0              # terrain surface height (m)

SUSPENSION_REF_HEIGHT = 0.7      # MAN chassis origin above wheel-bottom at rest (m)
TIRE_RADIUS = 0.585              # MAN 10t tire radius (m) for footprint assert
ZTOL = 0.10                      # allowed wheel-bottom clearance vs terrain top

init_x, init_y = 0.0, 0.0
init_z = terrain_top_z + SUSPENSION_REF_HEIGHT                # derived spawn height
init_loc = chrono.ChVector3d(init_x, init_y, init_z)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)


# === Data paths === locate bundled Chrono + vehicle assets (truth-faithful pair)
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# === Vehicle === MAN 10t catalog wrapper (owns its ChSystemNSC), TMEASY tires
vehicle = veh.MAN_10t()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)     # NSC for rigid terrain
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
vehicle.SetChassisFixed(False)                           # MANDATORY — fixed chassis won't move
vehicle.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
vehicle.SetTireType(veh.TireModelType_TMEASY)            # prompt: TMEASY tire model
vehicle.SetTireStepSize(time_step)
vehicle.Initialize()

# === System & bodies (created by the veh.MAN_10t wrapper) ===
system = vehicle.GetSystem()                             # ChSystemNSC owned by the wrapper
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED for contact, after Initialize
chassis = vehicle.GetChassisBody()                       # cache: main chassis rigid body, reused below
# wheels/spindles: vehicle.GetVehicle().GetSpindlePos(axle, side); joints: suspension + steering inside wrapper
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())  # report total vehicle mass

vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)

# Footprint check: wheel bottoms must rest on (not through) the terrain surface.
veh_obj = vehicle.GetVehicle()                           # cache: reused for spindle query
spindle_world = []
for axle in range(veh_obj.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        spindle_world.append(veh_obj.GetSpindlePos(axle, side))
wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS
assert wheel_bottom_z >= terrain_top_z - ZTOL, (
    f"vehicle sinks into terrain: wheel bottom z={wheel_bottom_z:.3f} "
    f"vs terrain top z={terrain_top_z:.3f}; raise SUSPENSION_REF_HEIGHT by "
    f"{terrain_top_z - wheel_bottom_z:.3f} m"
)

# === Terrain === flat rigid patch with a tiled texture under the truck
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, terrain_length, terrain_width)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Visualization === wheeled-vehicle Irrlicht: window + chase cam + sky + light + logo
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("MAN 10t on Rigid Terrain")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 2.0), 12.0, 1.0)   # follow point, distance, height
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()                                    # vehicle truths use a directional light
vis.AttachVehicle(vehicle.GetVehicle())

# === Driver === interactive keyboard driver bound to the visual system
driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0   # s to go 0 -> +1 steering
throttle_time = 1.0   # s to go 0 -> +1 throttle
braking_time = 0.3    # s to go 0 -> +1 brake
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# === Main loop === real-time Synchronize/Advance of the full subsystem stack
render_steps = math.ceil(render_step_size / time_step)       # precomputed once
realtime_timer = chrono.ChRealtimeStepTimer()


try:
    step_number = 0
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
        realtime_timer.Spin(time_step)
except (RuntimeError, ValueError) as exc:   # solver divergence / bad simulation state
    import traceback
    traceback.print_exc()
    raise

# === Post-processing === assemble review video + timeseries, prune frame dirs
