"""CityBus on flat rigid terrain, driven by a data-driven (pre-recorded) driver.

System: ChSystemNSC (rigid-terrain catalog vehicle, NSC contact).
Main bodies: the CityBus wrapper (chassis + suspensions + steering + 4 wheels/tires)
and a flat RigidTerrain patch the bus rolls on.
Driver: a veh.ChDataDriver fed a piecewise-linear schedule of (time, steering,
throttle, braking) entries — the bus stands still, then accelerates at full
throttle, then steers hard while still at full throttle.
Expected behavior: the bus pulls away from rest after t=0.1 s, builds speed under
full throttle, and curves to one side once steering ramps in at t=0.5 s.
"""

import math

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Parameters === geometry / timing constants (no bare literals downstream)
time_step = 1e-3                # integration step (s)
sim_end = 8.0                   # total simulated time (s)
render_fps = 50.0               # review-video frame rate
render_step_size = 1.0 / render_fps
render_steps = math.ceil(render_step_size / time_step)   # precomputed once

terrain_length = 200.0          # rigid patch X extent (m)
terrain_width = 100.0           # rigid patch Y extent (m)
init_loc = chrono.ChVector3d(0, 0, 0.5)     # chassis spawn (above ground)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)


# === Data paths === locate bundled Chrono + vehicle assets (truth components)
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# === Vehicle === CityBus catalog wrapper owns its ChSystemNSC
bus = veh.CityBus()
bus.SetContactMethod(chrono.ChContactMethod_NSC)     # rigid terrain -> NSC
bus.SetChassisCollisionType(veh.CollisionType_NONE)
bus.SetChassisFixed(False)                           # MANDATORY — fixed chassis won't move
bus.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
bus.SetTireType(veh.TireModelType_TMEASY)            # rigid-terrain tire model
bus.SetTireStepSize(time_step)
bus.Initialize()

bus.SetChassisVisualizationType(veh.VisualizationType_MESH)
bus.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
bus.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
bus.SetWheelVisualizationType(veh.VisualizationType_MESH)
bus.SetTireVisualizationType(veh.VisualizationType_MESH)

# === System & bodies (created by the veh.CityBus wrapper) ===
system = bus.GetSystem()                              # ChSystemNSC owned by the wrapper
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED for contact
chassis = bus.GetChassisBody()                        # main chassis rigid body
# wheels/spindles: bus.GetVehicle().GetSpindlePos(axle, side); joints: suspension + steering inside wrapper
print("VEHICLE MASS: ", bus.GetVehicle().GetMass())

# Footprint check: wheels must rest on (not through) the ground plane.
TIRE_RADIUS = 0.4665            # CityBus tire radius (m), from wheel geometry
GROUND_TOP_Z = 0.0
ZTOL = 0.1
veh_obj = bus.GetVehicle()
spindle_world = [veh_obj.GetSpindlePos(a, s)
                 for a in range(veh_obj.GetNumberAxles())
                 for s in (veh.LEFT, veh.RIGHT)]
wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS
assert wheel_bottom_z >= GROUND_TOP_Z - ZTOL, (
    f"bus sinks into ground: wheel bottom z={wheel_bottom_z:.3f} "
    f"vs ground top z={GROUND_TOP_Z:.3f}; raise init_loc.z"
)

# === Terrain === flat rigid patch the bus rolls on
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, terrain_length, terrain_width)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Visualization === full vehicle-aware Irrlicht scene (window + sky + camera + lights)
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("CityBus — data-driven driver")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 12.0, 0.7)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(bus.GetVehicle())

# === Driver === data-driven (pre-recorded) input schedule replacing the interactive driver
# Each entry is (time, steering, throttle, braking); inputs interpolate piecewise-linearly.
driver_data = veh.vector_Entry([
    veh.DataDriverEntry(0.0, 0.0, 0.0, 0.0),   # rest
    veh.DataDriverEntry(0.1, 0.0, 1.0, 0.0),   # full throttle, straight
    veh.DataDriverEntry(0.5, 0.7, 1.0, 0.0),   # full throttle, hard steer
])
driver = veh.ChDataDriver(bus.GetVehicle(), driver_data)
driver.Initialize()

# === Main loop === advance driver/terrain/vehicle/vis stack in real time
chassis_ref = chassis            # cache: chassis handle reused every step
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0


try:
    while vis.Run() and system.GetChTime() < sim_end:
        time = system.GetChTime()

        if step_number % render_steps == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()

        driver.Synchronize(time)
        terrain.Synchronize(time)
        bus.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)


        driver.Advance(time_step)
        terrain.Advance(time_step)
        bus.Advance(time_step)
        vis.Advance(time_step)

        step_number += 1
        realtime_timer.Spin(time_step)
except (RuntimeError, ValueError) as exc:   # solver divergence / invalid state
    import traceback; traceback.print_exc()
    raise
