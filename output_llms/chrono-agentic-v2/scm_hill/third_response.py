"""
HMMWV on Rigid Terrain with Heightmap Hill.

Models an HMMWV_Full driving over a bump/hill terrain defined by a BMP heightmap
loaded as a rigid terrain patch with NSC contact. The terrain uses a single
heightmap patch with a dirt texture; no deformable/SCM soil is used. The vehicle
uses NSC contact and TMEASY tires for traction.

System type: NSC (ChSystemNSC, owned by the HMMWV_Full wrapper)
Main bodies: HMMWV chassis, 4 wheel spindles, rigid terrain patch
Expected behavior: HMMWV accelerates and drives over the bump/hill terrain.
"""

import math
import os

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


# === Constants ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())          # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')      # locate vehicle data files

STEP_SIZE         = 1e-3        # physics time step (s)
SIM_END           = 20.0        # simulation end time (s)
RENDER_FPS        = 50.0
RENDER_STEPS      = math.ceil(1.0 / (RENDER_FPS * STEP_SIZE))  # precomputed once

TERRAIN_LENGTH    = 40.0        # terrain patch length (m)
TERRAIN_WIDTH     = 40.0        # terrain patch width (m)
TERRAIN_HEIGHT_MIN = -1.0       # heightmap min elevation (m)
TERRAIN_HEIGHT_MAX = 1.0        # heightmap max elevation (m)
TERRAIN_RES       = 0.02        # heightmap mesh resolution (m)

INIT_LOC          = chrono.ChVector3d(-15.0, 0.0, 1.0)  # vehicle spawn location
INIT_ROT          = chrono.ChQuaterniond(1, 0, 0, 0)

# === Vehicle setup ===
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)   # NSC for rigid terrain
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)                          # MANDATORY — fixed chassis won't move
hmmwv.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
hmmwv.SetTireType(veh.TireModelType_TMEASY)           # TMEASY for traction on rigid terrain
hmmwv.SetTireStepSize(STEP_SIZE)
hmmwv.Initialize()

# === System & bodies (created by the veh.HMMWV_Full wrapper) ===
sys = hmmwv.GetSystem()                  # ChSystemNSC owned by the wrapper
chassis = hmmwv.GetChassisBody()         # cache: fetched once, reused every step
# wheels/spindles: hmmwv.GetVehicle().GetAxle(i); terrain: RigidTerrain patch below
# joints: suspension + steering links created inside the wrapper

sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED after Initialize

print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())

hmmwv.SetChassisVisualizationType(chrono.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(chrono.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(chrono.VisualizationType_MESH)

# === Terrain — rigid terrain with heightmap patch (NSC contact) ===
terrain = veh.RigidTerrain(sys)

patch_mat = chrono.ChContactMaterialNSC()  # NSC to match vehicle contact method
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    veh.GetDataFile("terrain/height_maps/bump64.bmp"),
    TERRAIN_LENGTH,
    TERRAIN_WIDTH,
    TERRAIN_HEIGHT_MIN,
    TERRAIN_HEIGHT_MAX,
)
patch.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 6.0, 6.0)

terrain.Initialize()

# === Visualization — ChWheeledVehicleVisualSystemIrrlicht ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV on Rigid Heightmap Terrain")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()       # vehicle truths use directional light
vis.AttachVehicle(hmmwv.GetVehicle())

# === Driver — interactive (scored-core truth shape) ===
driver = veh.ChInteractiveDriverIRR(vis)

steering_time = 1.0
throttle_time = 1.0
braking_time  = 0.3
render_step_size = 1.0 / RENDER_FPS  # precomputed once

driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# === Recording setup ===


# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

try:
    while vis.Run() and hmmwv.GetSystem().GetChTime() < SIM_END:
        time = hmmwv.GetSystem().GetChTime()

        if step_number % RENDER_STEPS == 0:
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
        hmmwv.Advance(STEP_SIZE)    # advances the wrapper-owned ChSystem
        vis.Advance(STEP_SIZE)

        step_number += 1
        realtime_timer.Spin(STEP_SIZE)

except (RuntimeError, ValueError) as exc:   # solver divergence / bad vehicle state
    import traceback
    traceback.print_exc()
    raise
