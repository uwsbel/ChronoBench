"""Kraz tractor-trailer truck and a sedan on a highway-mesh rigid terrain.

System type: NSC (rigid-terrain catalog vehicles default to NSC contact).
Main bodies:
  - Kraz truck: a tractor + trailer wheeled vehicle (RIGID tires) initialized at a
    chosen world location/orientation; driven interactively (steering/throttle/brake).
  - Sedan: a BMW_E90 wheeled vehicle built on the SAME ChSystem, driven forward by a
    second, independent driver holding a fixed throttle and steering.
Terrain: a single RigidTerrain patch loaded from the predefined highway mesh.
Expected behavior: the sedan rolls forward under its constant throttle while the truck
sits/maneuvers on the same highway; both vehicles share the world, terrain, and contact.
"""

import os
import math

import pychrono.core as chrono
import pychrono.vehicle as veh


# === Constants === geometry / physics / timing (no bare literals downstream)
time_step = 2e-3                      # integration step
tire_step_size = 1e-3                 # tire force-model substep
sim_end = 8.0                         # bounded recording run length (sedan stays on road)
render_fps = 50.0
render_step_size = 1.0 / render_fps
render_steps = math.ceil(render_step_size / time_step)   # precomputed once

# The highway mesh runs lengthwise along +Y (roadway X-width is roughly [-11, 11]),
# so the travel direction is +Y. Both vehicles face +Y (yaw +90 deg) in separate
# lanes (truck at x=-3, sedan at x=+3) starting near the south end of the road.
ALONG_ROAD_YAW = math.pi / 2.0                           # face +Y (highway length axis)

# Truck (Kraz) initial pose — chosen location and orientation in the left lane.
TRUCK_INIT_LOC = chrono.ChVector3d(-3.0, -30.0, 0.5)
TRUCK_INIT_ROT = chrono.QuatFromAngleZ(ALONG_ROAD_YAW)

# Sedan initial pose — its own location and orientation in the right lane.
SEDAN_INIT_LOC = chrono.ChVector3d(3.0, -30.0, 0.4)
SEDAN_INIT_ROT = chrono.QuatFromAngleZ(ALONG_ROAD_YAW)

# Sedan open-loop command (fixed forward drive).
SEDAN_THROTTLE = 0.3
SEDAN_STEERING = 0.0

HIGHWAY_MESH = chrono.GetChronoDataFile("synchrono/meshes/Highway_vis.obj")

# === Data paths (truth components — locate bundled Chrono + vehicle assets) ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# === Truck (Kraz tractor-trailer) — owns the shared ChSystem ===
# The Kraz model uses RIGID tires (its built-in tire model); the truck therefore
# runs on rigid tires as requested.
truck = veh.Kraz()
truck.SetContactMethod(chrono.ChContactMethod_NSC)       # NSC for rigid terrain
truck.SetChassisCollisionType(veh.CollisionType_NONE)
truck.SetChassisFixed(False)                             # MANDATORY — fixed chassis won't move
truck.SetInitPosition(chrono.ChCoordsysd(TRUCK_INIT_LOC, TRUCK_INIT_ROT))
truck.SetTireStepSize(tire_step_size)
truck.Initialize()

truck.SetChassisVisualizationType(veh.VisualizationType_MESH, veh.VisualizationType_MESH)
truck.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
truck.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES, veh.VisualizationType_PRIMITIVES)
truck.SetWheelVisualizationType(veh.VisualizationType_MESH, veh.VisualizationType_MESH)
truck.SetTireVisualizationType(veh.VisualizationType_MESH, veh.VisualizationType_MESH)

# === System & bodies (created by the veh.Kraz wrapper) ===
system = truck.GetSystem()                               # ChSystemNSC owned by the truck wrapper
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED, after Initialize
truck_tractor = truck.GetTractor()                       # cache: tractor ChWheeledVehicle, reused
truck_chassis = truck.GetTractorChassisBody()            # cache: tractor chassis body, reused
print("VEHICLE MASS: ", truck.GetTractor().GetMass())

# === Sedan (BMW_E90) — built on the SHARED truck system, not a fresh one ===
sedan = veh.BMW_E90(truck.GetSystem())                   # shared system: same world/terrain/contact
sedan.SetChassisCollisionType(veh.CollisionType_NONE)
sedan.SetChassisFixed(False)
sedan.SetInitPosition(chrono.ChCoordsysd(SEDAN_INIT_LOC, SEDAN_INIT_ROT))
sedan.SetTireType(veh.TireModelType_TMEASY)              # sedan rolls on a slip-capable tire
sedan.SetTireStepSize(tire_step_size)
sedan.Initialize()

sedan.SetChassisVisualizationType(veh.VisualizationType_MESH)
sedan.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
sedan.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
sedan.SetWheelVisualizationType(veh.VisualizationType_MESH)
sedan.SetTireVisualizationType(veh.VisualizationType_MESH)
sedan_vehicle = sedan.GetVehicle()                       # cache: sedan ChWheeledVehicle, reused
print("SEDAN MASS: ", sedan.GetVehicle().GetMass())

# === Terrain === single rigid patch loaded from the predefined highway mesh
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, HIGHWAY_MESH)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
terrain.Initialize()

# === Visualization === full Irrlicht vehicle scene: window + sky + camera + lights
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("Kraz truck + sedan on highway")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 14.0, 1.0)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()                                # vehicle truths use a directional light
vis.AttachVehicle(truck.GetTractor())                    # Kraz attaches via the tractor

# === Drivers === interactive driver for the truck; a fixed-input driver for the sedan
truck_driver = veh.ChInteractiveDriverIRR(vis)           # truth-faithful interactive truck driver
truck_driver.SetSteeringDelta(render_step_size / 1.0)
truck_driver.SetThrottleDelta(render_step_size / 1.0)
truck_driver.SetBrakingDelta(render_step_size / 0.3)
truck_driver.Initialize()

sedan_driver_inputs = veh.DriverInputs()                 # second driver: holds fixed sedan command
sedan_driver_inputs.m_steering = SEDAN_STEERING
sedan_driver_inputs.m_throttle = SEDAN_THROTTLE
sedan_driver_inputs.m_braking = 0.0

# === Footprint check === spindle Z must rest on (not through) the highway surface
truck_obj = truck.GetTractor()
truck_wheel_bottom = min(
    truck_obj.GetSpindlePos(axle, side).z
    for axle in range(truck_obj.GetNumberAxles())
    for side in (veh.LEFT, veh.RIGHT)
)
assert truck_wheel_bottom > -1.0, (
    f"truck wheels spawned below the highway surface: z={truck_wheel_bottom:.3f}"
)

# === Main loop === real-time render-throttled stepping of the full subsystem stack
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
try:
    while vis.Run() and system.GetChTime() < sim_end:
        time = system.GetChTime()

        if step_number % render_steps == 0:             # throttled rendering
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        truck_inputs = truck_driver.GetInputs()


        # Truck subsystem synchronize (driver -> terrain -> vehicle -> vis).
        truck_driver.Synchronize(time)
        terrain.Synchronize(time)
        truck.Synchronize(time, truck_inputs, terrain)
        vis.Synchronize(time, truck_inputs)

        # Sedan subsystem synchronize with its own fixed driver inputs.
        sedan.Synchronize(time, sedan_driver_inputs, terrain)

        # Advance everything by one step.
        truck_driver.Advance(time_step)
        terrain.Advance(time_step)
        truck.Advance(time_step)                        # advances the wrapper-owned system
        sedan.Advance(time_step)
        vis.Advance(time_step)

        step_number += 1
        realtime_timer.Spin(time_step)
except (RuntimeError, ValueError) as exc:               # solver divergence / bad state
    import traceback; traceback.print_exc()
    raise
finally:
    pass
