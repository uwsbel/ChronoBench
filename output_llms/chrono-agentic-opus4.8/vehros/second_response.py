"""HMMWV full-model wheeled vehicle driving on flat rigid terrain (Irrlicht).

Models an HMMWV_Full catalog vehicle (NSC contact, Bullet collision) resting on a
single flat RigidTerrain patch textured with tile4.jpg. The chassis, suspension,
steering, wheels, and tires are all rendered with explicit visualization types.
An interactive Irrlicht driver feeds steering/throttle/braking inputs to the
vehicle. Expected behavior: the four wheels rest on the terrain at spawn and the
vehicle responds to driver inputs while the chase camera follows the chassis.
"""

import math
import pychrono.core as chrono
import pychrono.vehicle as veh
from pychrono import irrlicht as chronoirr

# === Constants === geometry / physics / timing (no bare literals downstream)
time_step = 1e-3                 # integration step (s)
sim_end = 12.0                   # bounded recording horizon (s)
render_fps = 50.0                # review render cadence
TIRE_RADIUS = 0.47               # HMMWV tire radius (m), for footprint assert
TERRAIN_LENGTH = 200.0           # rigid patch X extent (m)
TERRAIN_WIDTH = 200.0            # rigid patch Y extent (m)
TERRAIN_TOP_Z = 0.0              # terrain top surface height (m)
SUSPENSION_REF_HEIGHT = 0.5      # chassis origin above wheel-bottom at rest (m)
ZTOL = 0.1                       # allowed wheel-bottom clearance vs terrain top

init_loc = chrono.ChVector3d(0, 0, TERRAIN_TOP_Z + SUSPENSION_REF_HEIGHT)  # precomputed once
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)

# === Data paths === locate bundled Chrono + vehicle assets (truth components)
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# === Vehicle === HMMWV_Full owns its ChSystemNSC (NSC for rigid terrain)
vehicle = veh.HMMWV_Full()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
vehicle.SetChassisFixed(False)                       # MANDATORY — fixed chassis won't move
vehicle.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
vehicle.SetTireType(veh.TireModelType_TMEASY)        # rigid-terrain compatible tire model
vehicle.SetTireStepSize(time_step)
vehicle.Initialize()

# Visualization types (after Initialize) — chassis/suspension/steering/wheels/tires
vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)

# === System & bodies (created by the veh.HMMWV_Full wrapper) ===
system = vehicle.GetSystem()                          # ChSystemNSC owned by the wrapper
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED for contact
chassis = vehicle.GetChassisBody()                    # cache: main chassis rigid body, reused
# spindles: vehicle.GetVehicle().GetSpindlePos(axle, side); terrain patch body built below
# joints: suspension + steering links created inside the wrapper
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())   # report total vehicle mass

# Footprint sanity: wheel bottoms must rest on (not through) the terrain.
veh_obj = vehicle.GetVehicle()
spindle_world = []
for axle in range(veh_obj.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        spindle_world.append(veh_obj.GetSpindlePos(axle, side))
wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS
assert wheel_bottom_z >= TERRAIN_TOP_Z - ZTOL, (
    f"vehicle sinks into terrain: wheel bottom z={wheel_bottom_z:.3f} "
    f"vs terrain top z={TERRAIN_TOP_Z:.3f}; raise SUSPENSION_REF_HEIGHT"
)

# === Terrain === flat rigid patch textured with tile4.jpg
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 100, 100)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Visualization === vehicle-aware Irrlicht window + sky + camera + lights
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("HMMWV on rigid terrain")
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()                            # vehicle truths use a directional light
vis.AttachVehicle(vehicle.GetVehicle())

# === Driver === interactive Irrlicht driver bound to the visual system
render_step_size = 1.0 / render_fps
driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(render_step_size / 1.0)      # s to reach full steering
driver.SetThrottleDelta(render_step_size / 1.0)      # s to reach full throttle
driver.SetBrakingDelta(render_step_size / 0.3)       # s to reach full brake
driver.Initialize()

# === Main loop === real-time Synchronize/Advance over driver, terrain, vehicle, vis
render_steps = math.ceil(render_step_size / time_step)   # precomputed once
realtime_timer = chrono.ChRealtimeStepTimer()

step_number = 0
frame = 0
try:
    while vis.Run() and system.GetChTime() < sim_end:
        time = system.GetChTime()

        if step_number % render_steps == 0:          # throttled rendering
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
        vehicle.Advance(time_step)                   # advances the wrapper-owned system
        vis.Advance(time_step)

        step_number += 1
        realtime_timer.Spin(time_step)               # spin so wall-clock matches sim time
except (RuntimeError, ValueError) as exc:            # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
