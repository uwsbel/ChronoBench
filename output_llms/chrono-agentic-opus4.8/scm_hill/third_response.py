"""HMMWV full vehicle driving over a rigid heightmap terrain.

System type : NSC (non-smooth contact), Bullet collision.
Main bodies : HMMWV_Full wheeled vehicle (chassis, four spindles, tires) and a
              single rigid terrain patch built from a bump height map with a
              dirt texture.
Behavior    : The vehicle is driven forward over the undulating rigid hill; the
              wheels follow the heightmap surface (rigid contact, no soil
              deformation). An interactive driver controls steering/throttle/
              braking in real time.
"""

import math

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Parameters === geometry / physics constants and derived spawn pose
chrono.SetChronoDataPath(chrono.GetChronoDataPath())          # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')      # locate vehicle data files

step_size = 2e-3                       # integration step (s)
sim_end = 8.0                          # bounded recording horizon (s)
render_fps = 50.0                      # review render cadence (Hz)

TERRAIN_LENGTH = 100.0                 # heightmap patch X extent (m)
TERRAIN_WIDTH = 100.0                  # heightmap patch Y extent (m)
HMIN = -1.0                            # heightmap min elevation (m)
HMAX = 1.0                             # heightmap max elevation (m)

INIT_X = -10.0                         # spawn behind the hill crest
INIT_Y = 0.0
SUSPENSION_REF_HEIGHT = 0.5            # chassis origin above wheel-bottom at rest
TIRE_RADIUS = 0.46                     # HMMWV tire radius (m), for footprint assert
ZTOL = 0.25                            # wheel-bottom clearance tolerance on the hill
init_loc = chrono.ChVector3d(INIT_X, INIT_Y, HMAX + SUSPENSION_REF_HEIGHT)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)

# === Vehicle === HMMWV_Full wrapper owns its ChSystem (NSC) and all sub-bodies
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)   # NSC contact for the rigid terrain
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)                         # MANDATORY — fixed chassis won't move
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
hmmwv.SetTireType(veh.TireModelType_TMEASY)          # deformable-force tire, rolls on rigid hill
hmmwv.SetTireStepSize(step_size)
hmmwv.Initialize()

hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

# System & bodies created internally by the wrapper, surfaced into named locals:
system = hmmwv.GetSystem()                            # ChSystemNSC owned by the wrapper
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED, after Initialize
chassis = hmmwv.GetChassisBody()                      # cache: main chassis rigid body, reused below
# spindles: hmmwv.GetVehicle().GetSpindlePos(axle, side); joints: suspension/steering inside wrapper
print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())  # report total vehicle mass

# Footprint sanity: all four wheel bottoms must start on/above the hill surface.
veh_obj = hmmwv.GetVehicle()
spindle_world = [veh_obj.GetSpindlePos(axle, side)
                 for axle in range(veh_obj.GetNumberAxles())
                 for side in (veh.LEFT, veh.RIGHT)]
wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS
assert wheel_bottom_z >= HMAX - ZTOL, (
    f"vehicle starts below hill crest: wheel bottom z={wheel_bottom_z:.3f} "
    f"vs crest z={HMAX:.3f}; raise SUSPENSION_REF_HEIGHT"
)

# === Terrain === single rigid patch from a bump height map with a dirt texture
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()             # NSC material to match the system
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM,                                  # centered at origin, no rotation
    veh.GetDataFile("terrain/height_maps/bump64.bmp"),
    TERRAIN_LENGTH, TERRAIN_WIDTH, HMIN, HMAX,
)
patch.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 16, 16)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Visualization === vehicle-aware Irrlicht window: chase cam + sky + light
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV on Rigid Hill Terrain")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 8.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()                             # vehicle truths use a directional light
vis.AttachVehicle(hmmwv.GetVehicle())

# === Driver === interactive (keyboard) driver bound to the visual system
render_step_size = 1.0 / render_fps
driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(render_step_size / 1.0)       # 1 s to full steering
driver.SetThrottleDelta(render_step_size / 1.0)       # 1 s to full throttle
driver.SetBrakingDelta(render_step_size / 0.3)        # 0.3 s to full brake
driver.Initialize()

# === Main loop === real-time Synchronize/Advance over the full subsystem stack
render_steps = math.ceil(render_step_size / step_size)   # precomputed once

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
        hmmwv.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)


        driver.Advance(step_size)
        terrain.Advance(step_size)
        hmmwv.Advance(step_size)                      # advances the wrapper-owned system
        vis.Advance(step_size)

        step_number += 1
        realtime_timer.Spin(step_size)               # match wall-clock to sim time
except (RuntimeError, ValueError) as exc:            # solver divergence / bad state
    import traceback; traceback.print_exc()
    raise

# === Post-processing === assemble review video + physics plot, drop frame dirs
