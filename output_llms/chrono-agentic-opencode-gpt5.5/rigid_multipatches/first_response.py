"""HMMWV on multi-patch rigid terrain using PyChrono NSC contact.

The simulation builds a full HMMWV with mesh visualization, SHAFTS powertrain,
AWD driveline, and an interactive Irrlicht driver.  The rigid terrain contains
multiple flat textured patches plus a mesh bump and a heightmap elevation patch;
the vehicle is expected to drive forward over the sequence while the real-time
Irrlicht window follows the chassis.
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
SIM_END = 18.0
RENDER_FPS = 50.0
RENDER_STEP_SIZE = 1.0 / RENDER_FPS
RENDER_STEPS = math.ceil(RENDER_STEP_SIZE / STEP_SIZE)  # precomputed once

INIT_LOC = chrono.ChVector3d(-42.0, 0.0, 0.6)
INIT_ROT = chrono.QUNIT
TIRE_RADIUS = 0.469
WHEEL_Z_TOL = 0.15


# === Vehicle === wrapper creates the NSC system and all HMMWV rigid bodies
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)
hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
hmmwv.SetTireType(veh.TireModelType_TMEASY)
hmmwv.SetTireStepSize(TIRE_STEP_SIZE)
hmmwv.Initialize()

system = hmmwv.GetSystem()  # cache: wrapper-owned ChSystemNSC reused by terrain and logs
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
chassis = hmmwv.GetChassisBody()  # cache: chassis body reused for placement checks and logging
vehicle = hmmwv.GetVehicle()  # cache: vehicle interface reused for spindles, mass, and visualizer
# Wrapper-created essentials: chassis, suspension links, steering links, wheels,
# tires, driveline, and powertrain are owned by veh.HMMWV_Full.
print("VEHICLE MASS: ", vehicle.GetMass())

hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

spindle_world = []
for axle_index in range(vehicle.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        spindle_world.append(vehicle.GetSpindlePos(axle_index, side))
wheel_bottom_z = min(pos.z for pos in spindle_world) - TIRE_RADIUS
assert wheel_bottom_z >= -WHEEL_Z_TOL, (
    f"vehicle wheel bottom z={wheel_bottom_z:.3f} starts below terrain tolerance"
)


# === Terrain === multi-patch rigid terrain with flat, mesh, and heightmap surfaces
terrain = veh.RigidTerrain(system)

asphalt_mat = chrono.ChContactMaterialNSC()
asphalt_mat.SetFriction(0.90)
asphalt_mat.SetRestitution(0.01)
dirt_mat = chrono.ChContactMaterialNSC()
dirt_mat.SetFriction(0.75)
dirt_mat.SetRestitution(0.01)
grass_mat = chrono.ChContactMaterialNSC()
grass_mat.SetFriction(0.60)
grass_mat.SetRestitution(0.01)
bump_mat = chrono.ChContactMaterialNSC()
bump_mat.SetFriction(0.85)
bump_mat.SetRestitution(0.01)

patch = terrain.AddPatch(
    asphalt_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(-32.0, 0.0, 0.0), chrono.QUNIT),
    24.0,
    12.0,
)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 8.0, 8.0)

patch = terrain.AddPatch(
    dirt_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(-8.0, 0.0, 0.0), chrono.QUNIT),
    24.0,
    12.0,
)
patch.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 8.0, 8.0)

patch = terrain.AddPatch(
    grass_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(16.0, 0.0, 0.0), chrono.QUNIT),
    24.0,
    12.0,
)
patch.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 8.0, 8.0)

bump_patch = terrain.AddPatch(
    bump_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(38.0, 0.0, 0.0), chrono.QUNIT),
    veh.GetDataFile("terrain/meshes/bump.obj"),
)
bump_patch.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 4.0, 4.0)

height_patch = terrain.AddPatch(
    dirt_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(62.0, 0.0, 0.0), chrono.QUNIT),
    veh.GetDataFile("terrain/height_maps/bump64.bmp"),
    24.0,
    12.0,
    -0.2,
    0.8,
)
height_patch.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 8.0, 8.0)

terrain.Initialize()


# === Visualization & driver === Irrlicht vehicle window plus interactive controls
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV on rigid multi-patch terrain")
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


# === Main loop === synchronize driver, terrain, vehicle, and Irrlicht at a fixed step
step_number = 0
try:
    while vis.Run() and system.GetChTime() < SIM_END:
        if step_number % RENDER_STEPS == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        time = system.GetChTime()
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
except (RuntimeError, ValueError) as exc:
    print(f"simulation failed during vehicle dynamics: {exc}")
    raise
finally:
    pass
