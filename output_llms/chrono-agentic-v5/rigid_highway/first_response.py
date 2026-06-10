"""Full HMMWV wheeled vehicle on a custom highway mesh terrain (PyChrono + Irrlicht).

Models a complete HMMWV (chassis, suspension, steering, wheels, TMEASY tires)
driving on a custom rigid terrain whose collision and visual surfaces are loaded
from Wavefront meshes (Highway_col.obj / Highway_vis.obj). The vehicle uses the
NSC contact method (rigid terrain) and is steered through an interactive driver
(keyboard steering / throttle / braking). The scene renders in real time at
50 frames per second; every vehicle component is drawn with its mesh asset.

Expected behavior: the HMMWV rests on the highway surface and responds to driver
inputs (accelerating, braking, and steering) while the chase camera follows it.
"""

import math

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Parameters === geometry / physics / timing constants (no bare literals downstream)
STEP_SIZE = 2e-3                 # integration time step (s)
SIM_END = 20.0                   # bounded run length for the recording (s)
RENDER_FPS = 50.0                # real-time render cadence requested by the task
TIRE_STEP_SIZE = 1e-3            # TMEASY tire substep
INIT_X, INIT_Y, INIT_Z = 0.0, -60.0, 0.8  # HMMWV chassis spawn on the road (origin = geometric center)
INIT_HEADING = math.pi / 2       # initial yaw: face +Y, the highway's long axis (rad)

COLLISION_MESH = "synchrono/meshes/Highway_col.obj"   # rigid collision surface
VISUAL_MESH = "synchrono/meshes/Highway_vis.obj"      # rendered road surface

# === Data paths === locate bundled Chrono + vehicle assets (truth-faithful)
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# === Vehicle === full HMMWV with TMEASY tires on rigid terrain (NSC contact)
init_loc = chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z)
init_rot = chrono.QuatFromAngleZ(INIT_HEADING)

hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)   # NSC matches rigid-terrain catalog vehicles
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)                         # MANDATORY — a fixed chassis never moves
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
hmmwv.SetTireType(veh.TireModelType_TMEASY)          # prompt: TMEASY tire model
hmmwv.SetTireStepSize(TIRE_STEP_SIZE)
hmmwv.Initialize()

# Mesh visualization for all vehicle components (chassis, suspension, wheels, tires).
hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

# === System & bodies (created by the veh.HMMWV_Full wrapper) ===
system = hmmwv.GetSystem()                           # ChSystemNSC owned by the wrapper
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED for contact, after Initialize
chassis = hmmwv.GetChassisBody()                     # cache: main chassis rigid body, reused below
# wheels/spindles created inside the wrapper; suspension + steering joints likewise
print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())

# === Terrain === custom highway: rigid collision mesh + separate visual mesh
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM,                                 # mesh carries its own world placement
    chrono.GetChronoDataFile(COLLISION_MESH),        # Highway_col.obj — collision surface
)
terrain.Initialize()

# Attach the visual highway mesh to the patch ground body so the road is rendered.
vis_trimesh = chrono.ChTriangleMeshConnected().CreateFromWavefrontFile(
    chrono.GetChronoDataFile(VISUAL_MESH), True, True)   # Highway_vis.obj — load normals + UVs
vis_shape = chrono.ChVisualShapeTriangleMesh()
vis_shape.SetMesh(vis_trimesh)
vis_shape.SetName("highway_vis")
vis_shape.SetMutable(False)
patch.GetGroundBody().AddVisualShape(vis_shape, chrono.ChFramed())

# === Visualization === full vehicle-aware Irrlicht scene (window + sky + camera + lights)
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV on Highway Mesh Terrain")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()                            # vehicle truths use a directional light
vis.AttachVehicle(hmmwv.GetVehicle())

# === Driver === interactive keyboard control of steering / throttle / braking
render_step_size = 1.0 / RENDER_FPS                  # precomputed once: real-time render period
driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(render_step_size / 1.0)      # time to reach full steering (s)
driver.SetThrottleDelta(render_step_size / 1.0)      # time to reach full throttle (s)
driver.SetBrakingDelta(render_step_size / 0.3)       # time to reach full braking (s)
driver.Initialize()

# === Main loop === real-time Synchronize/Advance of the full subsystem stack
render_steps = math.ceil(render_step_size / STEP_SIZE)   # precomputed once: steps per frame
realtime_timer = chrono.ChRealtimeStepTimer()


step_number = 0
try:
    while vis.Run() and system.GetChTime() < SIM_END:
        time = system.GetChTime()

        if step_number % render_steps == 0:          # throttled rendering at RENDER_FPS
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()


        driver.Synchronize(time)
        terrain.Synchronize(time)
        hmmwv.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        driver.Advance(STEP_SIZE)
        terrain.Advance(STEP_SIZE)
        hmmwv.Advance(STEP_SIZE)                      # advances the wrapper-owned system
        vis.Advance(STEP_SIZE)

        step_number += 1
        realtime_timer.Spin(STEP_SIZE)               # spin so wall-clock matches sim time
except (RuntimeError, ValueError) as exc:            # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise

# === Post-processing === assemble the review video + physics plots from the run
