"""HMMWV full-vehicle simulation on NSC rigid terrain with multiple patch types.

The scene uses PyChrono's HMMWV_Full wrapper, a Bullet-backed rigid terrain made
from textured flat patches, a mesh bump patch, and a heightmap patch. Irrlicht
renders the vehicle in real time while an interactive driver controls steering,
throttle, and braking.
"""

import math
import traceback

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


# === Constants ===
# Named parameters keep vehicle, terrain, and render rates easy to audit.
STEP_SIZE = 2.0e-3
TIRE_STEP_SIZE = STEP_SIZE
SIM_END = 8.0
RENDER_FPS = 50.0
RENDER_STEP_SIZE = 1.0 / RENDER_FPS
RENDER_STEPS = max(1, math.ceil(RENDER_STEP_SIZE / STEP_SIZE))  # precomputed once

INIT_LOC = chrono.ChVector3d(-36.0, 0.0, 0.55)
INIT_ROT = chrono.QUNIT
TIRE_RADIUS = 0.47
WHEEL_Z_TOL = 0.12

TERRAIN_LENGTH = 100.0
TERRAIN_WIDTH = 18.0
PATCH_LENGTH = 20.0
PATCH_WIDTH = 18.0


# === Vehicle ===
# The HMMWV wrapper owns its ChSystem; configure it before Initialize.
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)
hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
hmmwv.SetTireType(veh.TireModelType_RIGID)
hmmwv.SetTireStepSize(TIRE_STEP_SIZE)
hmmwv.Initialize()

system = hmmwv.GetSystem()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())

hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

vehicle = hmmwv.GetVehicle()  # cache: wrapper vehicle reused for queries and visualization
chassis = hmmwv.GetChassisBody()  # cache: chassis body reused for diagnostics
spindle_positions = []  # cache: initialized wheel positions used for spawn validation
for axle_index in range(vehicle.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        spindle_positions.append(vehicle.GetSpindlePos(axle_index, side))

wheel_bottom_z = min(pos.z for pos in spindle_positions) - TIRE_RADIUS
assert wheel_bottom_z >= -WHEEL_Z_TOL, (
    f"vehicle starts below terrain: wheel bottom z={wheel_bottom_z:.3f}"
)

# Wrapper-created essentials: owned system, chassis/suspension/steering/wheels,
# rigid tires, vehicle visualization, interactive driver, and rigid terrain.
assert chassis is not None


# === Terrain ===
# Build a single RigidTerrain object from diverse flat, mesh, and heightmap patches.
terrain = veh.RigidTerrain(system)

flat_mat_high_grip = chrono.ChContactMaterialNSC()
flat_mat_high_grip.SetFriction(0.9)
flat_mat_high_grip.SetRestitution(0.01)

flat_mat_mid_grip = chrono.ChContactMaterialNSC()
flat_mat_mid_grip.SetFriction(0.75)
flat_mat_mid_grip.SetRestitution(0.01)

bump_mat = chrono.ChContactMaterialNSC()
bump_mat.SetFriction(0.85)
bump_mat.SetRestitution(0.01)

height_mat = chrono.ChContactMaterialNSC()
height_mat.SetFriction(0.8)
height_mat.SetRestitution(0.01)

patch0 = terrain.AddPatch(
    flat_mat_high_grip,
    chrono.ChCoordsysd(chrono.ChVector3d(-30.0, 0.0, 0.0), chrono.QUNIT),
    PATCH_LENGTH,
    PATCH_WIDTH,
)
patch0.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 16, 8)
patch0.SetColor(chrono.ChColor(0.75, 0.75, 0.70))

patch1 = terrain.AddPatch(
    flat_mat_mid_grip,
    chrono.ChCoordsysd(chrono.ChVector3d(-10.0, 0.0, 0.0), chrono.QUNIT),
    PATCH_LENGTH,
    PATCH_WIDTH,
)
patch1.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 12, 8)
patch1.SetColor(chrono.ChColor(0.45, 0.35, 0.25))

mesh_patch = terrain.AddPatch(
    bump_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(14.0, 0.0, 0.0), chrono.QUNIT),
    veh.GetDataFile("terrain/meshes/bump.obj"),
)
mesh_patch.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 8, 8)
mesh_patch.SetColor(chrono.ChColor(0.50, 0.52, 0.50))

height_patch = terrain.AddPatch(
    height_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(38.0, 0.0, 0.0), chrono.QUNIT),
    veh.GetDataFile("terrain/height_maps/bump64.bmp"),
    24.0,
    PATCH_WIDTH,
    -0.20,
    0.60,
)
height_patch.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 10, 8)
height_patch.SetColor(chrono.ChColor(0.30, 0.45, 0.28))

patch4 = terrain.AddPatch(
    flat_mat_high_grip,
    chrono.ChCoordsysd(chrono.ChVector3d(64.0, 0.0, 0.0), chrono.QUNIT),
    32.0,
    PATCH_WIDTH,
)
patch4.SetTexture(veh.GetDataFile("terrain/textures/Concrete037_1K-JPG/Concrete037_1K_Color.jpg"), 14, 8)
patch4.SetColor(chrono.ChColor(0.62, 0.62, 0.60))

terrain.Initialize()


# === Visualization & Driver ===
# Vehicle-specific Irrlicht visualization provides the chase camera and input HUD.
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV rigid terrain multipatches")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 8.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(vehicle)

driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(RENDER_STEP_SIZE / 1.0)
driver.SetThrottleDelta(RENDER_STEP_SIZE / 1.0)
driver.SetBrakingDelta(RENDER_STEP_SIZE / 0.3)
driver.Initialize()

realtime_timer = chrono.ChRealtimeStepTimer()


# === Main loop ===
# Synchronize and advance the driver, terrain, vehicle, and visual system every step.


try:
    frame = 0
    step_number = 0
    while vis.Run() and system.GetChTime() < SIM_END:
        time = system.GetChTime()

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
        hmmwv.Advance(STEP_SIZE)
        vis.Advance(STEP_SIZE)

        step_number += 1
        realtime_timer.Spin(STEP_SIZE)
except (OSError, IOError) as exc:
    traceback.print_exc()
    raise
except (RuntimeError, ValueError) as exc:
    traceback.print_exc()
    raise
finally:
    pass
