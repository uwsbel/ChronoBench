"""CityBus on rigid terrain — interactive real-time driving demo (PyChrono + Irrlicht).

Models a CityBus catalog wheeled vehicle (NSC contact) initialized with a given
location/orientation and a TMEASY tire model, driven on a flat RigidTerrain patch
that carries a custom road texture. The vehicle uses a combination of mesh
visualization (chassis, wheels, tires) and primitive visualization (suspension,
steering) for its parts. An interactive keyboard driver (ChInteractiveDriverIRR)
controls steering / throttle / braking, and a chase camera follows the vehicle.
The loop runs at 50 rendered frames per second in real time, synchronizing and
advancing the full driver/terrain/vehicle/visual stack each physics step.
Expected behavior: the bus rests on the textured terrain and responds to driver
inputs (drives forward / steers) without sinking through the ground.
"""

import math

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Parameters === geometry / timing / initial pose constants (no bare literals below)
step_size = 1e-3                  # physics time step (s)
render_fps = 50.0                # rendered frames per second (prompt: 50 FPS)
render_step_size = 1.0 / render_fps
TERRAIN_LENGTH = 200.0           # rigid patch X extent (m)
TERRAIN_WIDTH = 200.0            # rigid patch Y extent (m)
TIRE_RADIUS = 0.5               # approx CityBus tire radius (m) for footprint check
ZTOL = 0.10                      # allowed wheel-bottom clearance vs terrain top (m)
TERRAIN_TOP_Z = 0.0              # flat patch top surface height (m)
INIT_LOC = chrono.ChVector3d(0, 0, 0.5)          # vehicle spawn location (m)
INIT_ROT = chrono.ChQuaterniond(1, 0, 0, 0)      # spawn orientation (identity)

# === Data paths === anchor the bundled Chrono + vehicle asset trees
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# === Vehicle === CityBus catalog wrapper (owns its ChSystemNSC) on rigid terrain
bus = veh.CityBus()
bus.SetContactMethod(chrono.ChContactMethod_NSC)     # NSC for rigid terrain
bus.SetChassisCollisionType(veh.CollisionType_NONE)
bus.SetChassisFixed(False)                           # MANDATORY — fixed chassis won't move
bus.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
bus.SetTireType(veh.TireModelType_TMEASY)            # prompt: tire model (TMEASY on rigid road)
bus.SetTireStepSize(step_size)
bus.Initialize()

# Mesh + primitive visualization combination for the different vehicle parts.
bus.SetChassisVisualizationType(veh.VisualizationType_MESH)          # chassis: mesh
bus.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)  # suspension: primitive
bus.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)    # steering: primitive
bus.SetWheelVisualizationType(veh.VisualizationType_MESH)            # wheels: mesh
bus.SetTireVisualizationType(veh.VisualizationType_MESH)             # tires: mesh

# === System & bodies (created by the veh.CityBus wrapper) ===
system = bus.GetSystem()                              # ChSystemNSC owned by the wrapper
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED for contact
chassis = bus.GetChassisBody()                        # main chassis rigid body
# spindles: bus.GetVehicle().GetSpindlePos(axle, side); joints: suspension + steering links
# (created inside the wrapper); terrain: RigidTerrain patch body built below.
print("VEHICLE MASS: ", bus.GetVehicle().GetMass())   # report total vehicle mass

# Validate the wheel footprint rests on (not through) the flat terrain top.
veh_obj = bus.GetVehicle()
spindle_world = []
for axle in range(veh_obj.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        spindle_world.append(veh_obj.GetSpindlePos(axle, side))
wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS
assert wheel_bottom_z >= TERRAIN_TOP_Z - ZTOL, (
    f"vehicle sinks into terrain: wheel bottom z={wheel_bottom_z:.3f} "
    f"vs terrain top z={TERRAIN_TOP_Z:.3f}; raise INIT_LOC.z"
)

# === Terrain === flat rigid patch with a custom road texture
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)  # custom texture
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Visualization === vehicle-aware Irrlicht window: chase camera + sky + lights
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("CityBus on Rigid Terrain")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 10.0, 0.5)   # follow from behind/above
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()                             # vehicle truths use a directional light
vis.AttachVehicle(bus.GetVehicle())

# === Driver === interactive keyboard driver bound to the visual system
driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0                                   # s to ramp steering 0 -> 1
throttle_time = 1.0                                   # s to ramp throttle 0 -> 1
braking_time = 0.3                                    # s to ramp braking 0 -> 1
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# === Main loop === real-time render-throttled driver/terrain/vehicle/vis stepping
render_steps = math.ceil(render_step_size / step_size)   # precomputed once
realtime_timer = chrono.ChRealtimeStepTimer()

step_number = 0
try:
    while vis.Run():
        time = system.GetChTime()

        if step_number % render_steps == 0:             # throttled rendering
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()


        driver.Synchronize(time)
        terrain.Synchronize(time)
        bus.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        driver.Advance(step_size)
        terrain.Advance(step_size)
        bus.Advance(step_size)
        vis.Advance(step_size)

        step_number += 1
        realtime_timer.Spin(step_size)                  # match wall-clock to sim time

except (RuntimeError, ValueError) as exc:               # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
