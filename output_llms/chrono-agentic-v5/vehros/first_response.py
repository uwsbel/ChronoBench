"""HMMWV-on-rigid-terrain simulation with a ROS2 bridge.

Models an HMMWV_Full wheeled vehicle (NSC contact, SHAFTS engine +
AUTOMATIC_SHAFTS transmission, TMEASY tires) driving on a flat RigidTerrain
patch with defined friction and restitution. An interactive driver provides
steering / throttle / braking. A ChROSPythonManager publishes the simulation
clock, subscribes to driver inputs, and publishes the chassis (vehicle) state
over ROS2, so an external ROS graph can both command and observe the vehicle.

System type: NSC (rigid terrain). Main bodies: HMMWV chassis + 4 wheels/tires,
rigid terrain patch. Expected behavior: the vehicle rests on the terrain and is
driven via ROS-subscribed / interactive inputs while its state streams to ROS.
"""

import math

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.ros as chros


# === Constants === geometry / physics / driver timing (no bare literals downstream)
TIME_STEP = 1.0e-3                 # integration step (s)
TIRE_STEP_SIZE = 1.0e-3            # tire force-model step (s)
SIM_END = 12.0                     # bounded recording-run end time (s)
RENDER_FPS = 50.0                  # review-video frame cadence

TERRAIN_LENGTH = 100.0            # terrain patch X size (m)
TERRAIN_WIDTH = 100.0            # terrain patch Y size (m)
TERRAIN_FRICTION = 0.9          # terrain Coulomb friction coefficient
TERRAIN_RESTITUTION = 0.01      # terrain restitution (bounciness)

INIT_LOC = chrono.ChVector3d(0, 0, 0.5)     # chassis-origin spawn (above terrain z=0)
INIT_ROT = chrono.ChQuaterniond(1, 0, 0, 0)  # identity orientation
TIRE_RADIUS = 0.46               # HMMWV tire radius (m) for wheel-bottom assertion
TERRAIN_TOP_Z = 0.0              # flat rigid terrain surface height (m)
ZTOL = 0.10                      # allowed wheel-bottom clearance/overlap vs terrain

render_step_size = 1.0 / RENDER_FPS                 # precomputed once: render cadence
render_steps = math.ceil(render_step_size / TIME_STEP)   # precomputed once: steps/frame

# === Data paths === anchor bundled Chrono + vehicle assets (truth-faithful)
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# === Vehicle === HMMWV_Full wrapper owns its ChSystem; configure powertrain + tires
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)        # NSC for rigid terrain
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)                              # MANDATORY — fixed chassis won't move
hmmwv.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)          # requested engine model
hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
hmmwv.SetTireType(veh.TireModelType_TMEASY)              # prompt: TMEASY tire model
hmmwv.SetTireStepSize(TIRE_STEP_SIZE)
hmmwv.Initialize()

hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

# === System & bodies (created by the veh.HMMWV_Full wrapper) ===
system = hmmwv.GetSystem()                               # ChSystemNSC owned by the wrapper
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED for contact
chassis = hmmwv.GetChassisBody()       # cache: main chassis rigid body, reused every step
chassis.SetName("chassis")             # name the body so its ROS topic/frame is meaningful
# wheels/spindles: hmmwv.GetVehicle().GetSpindlePos(axle, side); suspension + steering
# joints are created inside the wrapper. Terrain patch body is built below.
print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())   # report total vehicle mass

# === Terrain === flat rigid patch with the requested friction + restitution
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(TERRAIN_FRICTION)
patch_mat.SetRestitution(TERRAIN_RESTITUTION)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Assert the wheels rest on (not through) the terrain after Initialize.
veh_obj = hmmwv.GetVehicle()
spindle_world = [veh_obj.GetSpindlePos(axle, side)
                 for axle in range(veh_obj.GetNumberAxles())
                 for side in (veh.LEFT, veh.RIGHT)]
wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS
assert wheel_bottom_z >= TERRAIN_TOP_Z - ZTOL, (
    f"vehicle sinks into terrain: wheel bottom z={wheel_bottom_z:.3f} "
    f"vs terrain top z={TERRAIN_TOP_Z:.3f}; raise INIT_LOC.z"
)

# === Visualization === vehicle-aware Irrlicht window: sky + chase camera + light
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV with ROS bridge")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()                                # vehicle truths use a directional light
vis.AttachVehicle(hmmwv.GetVehicle())

# === Driver === interactive steering/throttle/braking bound to the visual system
driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(render_step_size / 1.0)          # seconds 0 -> +1 steering
driver.SetThrottleDelta(render_step_size / 1.0)          # seconds 0 -> +1 throttle
driver.SetBrakingDelta(render_step_size / 0.3)           # seconds 0 -> +1 brake
driver.Initialize()

# === ROS bridge === clock (pub) + driver inputs (sub) + chassis state (pub)
ros_manager = chros.ChROSPythonManager()
ros_manager.RegisterHandler(chros.ChROSClockHandler())   # /clock FIRST — time-sync the graph
ros_manager.RegisterHandler(                             # SUBSCRIBE throttle/steer/brake
    chros.ChROSDriverInputsHandler(25, driver, "~/input/driver_inputs"))
ros_manager.RegisterHandler(                             # PUBLISH chassis pose/twist
    chros.ChROSBodyHandler(25, chassis, "~/output/hmmwv/state"))
ros_manager.Initialize()                                 # exactly once, after all handlers

# === Main loop === real-time Synchronize/Advance stack; ROS update LAST each step
realtime_timer = chrono.ChRealtimeStepTimer()
driver_inputs = driver.GetInputs()      # cache: struct reused/refreshed each step


frame = 0
step_number = 0
try:
    while vis.Run() and system.GetChTime() < SIM_END:
        time = system.GetChTime()

        if step_number % render_steps == 0:             # throttled rendering
            vis.BeginScene()
            vis.Render()
            vis.EndScene()


        driver_inputs = driver.GetInputs()


        driver.Synchronize(time)
        terrain.Synchronize(time)
        hmmwv.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        driver.Advance(TIME_STEP)
        terrain.Advance(TIME_STEP)
        hmmwv.Advance(TIME_STEP)                         # advances the wrapper-owned system
        vis.Advance(TIME_STEP)

        if not ros_manager.Update(time, TIME_STEP):     # ROS update LAST; break on shutdown
            break

        step_number += 1
        realtime_timer.Spin(TIME_STEP)                  # keep wall-clock ~ sim time
except (RuntimeError, ValueError) as exc:               # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
