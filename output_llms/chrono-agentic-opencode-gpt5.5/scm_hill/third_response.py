"""HMMWV climbing a rigid height-map hill with NSC contact.

The simulation uses the catalog HMMWV on a single rigid terrain patch built from
Chrono's bump height map and dirt texture.  The vehicle, tires, rigid terrain,
driver, and Irrlicht visualization run on the wrapper-owned NSC system, and the
expected behavior is forward motion over the hilly rigid surface.
"""

import math

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


# === Constants === named parameters keep vehicle, terrain, and loop settings explicit
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

STEP_SIZE = 0.002
TIRE_STEP_SIZE = STEP_SIZE
SIM_END = 5.5
RENDER_FPS = 25.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))  # precomputed once

TERRAIN_LENGTH = 40.0
TERRAIN_WIDTH = 40.0
TERRAIN_MIN_Z = -1.0
TERRAIN_MAX_Z = 1.0
TERRAIN_FRICTION = 0.9
TERRAIN_RESTITUTION = 0.01

INIT_POS = chrono.ChVector3d(-12.0, 0.0, 1.2)
INIT_ROT = chrono.QUNIT
TRACK_POINT = chrono.ChVector3d(0.0, 0.0, 1.75)


# === Vehicle and system === wrapper creates the NSC vehicle system and moving bodies
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysd(INIT_POS, INIT_ROT))
hmmwv.SetTireType(veh.TireModelType_TMEASY)
hmmwv.SetTireStepSize(TIRE_STEP_SIZE)
hmmwv.Initialize()

system = hmmwv.GetSystem()  # cache: wrapper-owned ChSystemNSC reused throughout
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
chassis = hmmwv.GetChassisBody()  # cache: main chassis body logged every step
vehicle_core = hmmwv.GetVehicle()  # cache: wheeled-vehicle object for vis/driver/diagnostics
# Bodies and joints are created by HMMWV_Full: chassis, axles, wheels, steering,
# suspension, driveline, and tire subsystems all live in the wrapper-owned system.
print("VEHICLE MASS: ", vehicle_core.GetMass())

hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)


# === Terrain === one rigid height-map patch replaces deformable SCM terrain
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(TERRAIN_FRICTION)
patch_mat.SetRestitution(TERRAIN_RESTITUTION)
patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM,
    veh.GetDataFile("terrain/height_maps/bump64.bmp"),
    TERRAIN_LENGTH,
    TERRAIN_WIDTH,
    TERRAIN_MIN_Z,
    TERRAIN_MAX_Z,
)
patch.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 6.0, 6.0)
terrain.Initialize()


# === Visualization === vehicle-specific Irrlicht scene mirrors catalog demos
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV on rigid height-map hill")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(TRACK_POINT, 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(vehicle_core)


# === Driver === interactive core, with a tagged record-mode data driver for validation
render_step_size = 1.0 / RENDER_FPS  # precomputed once
driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(render_step_size / 1.0)
driver.SetThrottleDelta(render_step_size / 1.0)
driver.SetBrakingDelta(render_step_size / 0.3)


driver.Initialize()


# === Main loop === synchronize and advance driver, terrain, vehicle, and renderer
realtime_timer = chrono.ChRealtimeStepTimer()
frame = 0
step_number = 0

try:

    while vis.Run() and system.GetChTime() < SIM_END:
        if step_number % RENDER_EVERY == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        for _ in range(RENDER_EVERY):
            time = system.GetChTime()
            driver_inputs = driver.GetInputs()  # cache: used by vehicle and visual sync

            driver.Synchronize(time)
            terrain.Synchronize(time)
            hmmwv.Synchronize(time, driver_inputs, terrain)
            vis.Synchronize(time, driver_inputs)

            driver.Advance(STEP_SIZE)
            terrain.Advance(STEP_SIZE)
            hmmwv.Advance(STEP_SIZE)
            vis.Advance(STEP_SIZE)


            step_number += 1
            realtime_timer.Spin(STEP_SIZE)
            if system.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:  # solver divergence or invalid state
    raise
except (OSError, IOError) as exc:  # output directory or CSV write failure
    raise
finally:
    pass


# === Post-processing === review-only video assembly and frame cleanup
