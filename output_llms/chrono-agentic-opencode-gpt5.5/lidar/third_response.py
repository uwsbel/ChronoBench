"""ARTcar lidar sensor demo on rigid terrain.

This NSC vehicle simulation initializes an ARTcar, rigid terrain, two chassis-
mounted lidar sensors, and a chassis-mounted third-person camera. The vehicle
drives forward under a simple data driver while the sensor manager updates the
3D lidar, planar 2D lidar, and camera streams attached to the chassis.
"""

import math

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens
import pychrono.vehicle as veh


# === Constants === named setup values keep vehicle, terrain, and sensors reproducible
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

STEP_SIZE = 1.0e-3
TIRE_STEP_SIZE = STEP_SIZE
SIM_END = 8.0
RENDER_FPS = 30.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))  # precomputed once
TERRAIN_LENGTH = 220.0
TERRAIN_WIDTH = 20.0
ARTCAR_INIT_POS = chrono.ChVector3d(0.0, 0.0, 0.6)
ARTCAR_INIT_ROT = chrono.QUNIT
LIDAR_OFFSET = chrono.ChVector3d(1.0, 0.0, 1.0)
LIDAR_RATE = 5.0
CAMERA_RATE = 30.0


# === Vehicle & system === wrapper creates the ARTcar system, chassis, axles, and joints
vehicle = veh.ARTcar()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(ARTCAR_INIT_POS, ARTCAR_INIT_ROT))
vehicle.SetTireType(veh.TireModelType_RIGID)
vehicle.SetTireStepSize(TIRE_STEP_SIZE)
vehicle.Initialize()

system = vehicle.GetSystem()  # cache: wrapper-owned ChSystemNSC reused by terrain/sensors/loop
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
chassis = vehicle.GetChassisBody()  # cache: real chassis body used for all attached sensors
veh_core = vehicle.GetVehicle()  # cache: vehicle subsystem used for mass and driver setup
# Wrapper-created components: system, chassis, wheels, steering, suspension, and joints.
print("VEHICLE MASS: ", veh_core.GetMass())

vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)


# === Terrain === rigid flat patch gives the ARTcar contact support and visual ground
terrain = veh.RigidTerrain(system)
terrain_mat = chrono.ChContactMaterialNSC()
terrain_mat.SetFriction(0.9)
terrain_mat.SetRestitution(0.01)
patch = terrain.AddPatch(
    terrain_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    TERRAIN_LENGTH,
    TERRAIN_WIDTH,
)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 50)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()


# === Sensors === lidar and third-person camera ride on the vehicle chassis
manager = sens.ChSensorManager(system)
manager.scene.AddPointLight(
    chrono.ChVector3f(0, 0, 12),
    chrono.ChColor(1.0, 1.0, 1.0),
    80.0,
)
manager.scene.AddAreaLight(
    chrono.ChVector3f(0, 0, 6),
    chrono.ChColor(1.0, 1.0, 1.0),
    60.0,
    chrono.ChVector3f(4, 0, 0),
    chrono.ChVector3f(0, 4, 0),
)

lidar_pose = chrono.ChFramed(
    LIDAR_OFFSET,
    chrono.QuatFromAngleAxis(0.0, chrono.ChVector3d(0, 1, 0)),
)

lidar_3d = sens.ChLidarSensor(
    chassis,
    LIDAR_RATE,
    lidar_pose,
    800,
    300,
    2 * chrono.CH_PI,
    chrono.CH_PI / 12,
    -chrono.CH_PI / 6,
    100.0,
    sens.LidarBeamShape_RECTANGULAR,
    2,
    0.003,
    0.003,
    sens.LidarReturnMode_STRONGEST_RETURN,
)
lidar_3d.SetName("3D Lidar Sensor")
lidar_3d.SetLag(0)
lidar_3d.SetCollectionWindow(1.0 / LIDAR_RATE)
lidar_3d.PushFilter(sens.ChFilterVisualize(800, 300, "3D Raw Lidar Depth"))
lidar_3d.PushFilter(sens.ChFilterDIAccess())
lidar_3d.PushFilter(sens.ChFilterPCfromDepth())
lidar_3d.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "3D Lidar Point Cloud"))
lidar_3d.PushFilter(sens.ChFilterXYZIAccess())
manager.AddSensor(lidar_3d)

lidar_2d = sens.ChLidarSensor(
    chassis,
    LIDAR_RATE,
    lidar_pose,
    800,
    1,
    2 * chrono.CH_PI,
    0.0,
    0.0,
    100.0,
    sens.LidarBeamShape_RECTANGULAR,
    2,
    0.003,
    0.003,
    sens.LidarReturnMode_STRONGEST_RETURN,
)
lidar_2d.SetName("2D Lidar Sensor")
lidar_2d.SetLag(0)
lidar_2d.SetCollectionWindow(1.0 / LIDAR_RATE)
lidar_2d.PushFilter(sens.ChFilterVisualize(800, 1, "2D Raw Lidar Depth"))
lidar_2d.PushFilter(sens.ChFilterDIAccess())
lidar_2d.PushFilter(sens.ChFilterPCfromDepth())
lidar_2d.PushFilter(sens.ChFilterVisualizePointCloud(640, 240, 1.0, "2D Lidar Point Cloud"))
lidar_2d.PushFilter(sens.ChFilterXYZIAccess())
manager.AddSensor(lidar_2d)

camera_pose = chrono.ChFramed(
    chrono.ChVector3d(-4.0, 0.0, 2.0),
    chrono.QuatFromAngleAxis(0.18, chrono.ChVector3d(0, 1, 0)),
)
third_person_camera = sens.ChCameraSensor(chassis, CAMERA_RATE, camera_pose, 1280, 720, 1.408)
third_person_camera.SetName("Third Person Camera")
third_person_camera.SetLag(0)
third_person_camera.SetCollectionWindow(0)
third_person_camera.PushFilter(sens.ChFilterVisualize(1280, 720, "Third Person Camera"))
third_person_camera.PushFilter(sens.ChFilterRGBA8Access())
third_person_camera.PushFilter(sens.ChFilterSave("cam/third_person/"))
manager.AddSensor(third_person_camera)


# === Visualization & driver === Irrlicht follows the ARTcar while the driver supplies inputs
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("ARTcar Lidar")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.0), 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(veh_core)

driver_data = veh.vector_Entry([
    veh.DataDriverEntry(0.0, 0.0, 0.0, 0.0),
    veh.DataDriverEntry(0.5, 0.0, 0.35, 0.0),
    veh.DataDriverEntry(SIM_END, 0.0, 0.35, 0.0),
])
driver = veh.ChDataDriver(veh_core, driver_data)
driver.Initialize()


# === Main loop === update vehicle subsystems and sensor manager on the same system

frame = 0
try:
    realtime_timer = chrono.ChRealtimeStepTimer()
    while vis.Run() and system.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        for _ in range(RENDER_EVERY):
            time = system.GetChTime()
            driver_inputs = driver.GetInputs()
            pos = chassis.GetPos()  # cache: current chassis pose for logging and buffer guard
            vel = chassis.GetPosDt()  # cache: current chassis velocity for logging

            lidar_depth = lidar_3d.GetMostRecentDIBuffer()
            if lidar_depth.HasData():
                pass  # guard: lidar access buffer may be empty before the first sensor tick
            camera_buffer = third_person_camera.GetMostRecentRGBA8Buffer()
            if camera_buffer.HasData():
                pass  # guard: camera access buffer may be empty before the first sensor tick

            driver.Synchronize(time)
            terrain.Synchronize(time)
            vehicle.Synchronize(time, driver_inputs, terrain)
            vis.Synchronize(time, driver_inputs)

            driver.Advance(STEP_SIZE)
            terrain.Advance(STEP_SIZE)
            vehicle.Advance(STEP_SIZE)
            vis.Advance(STEP_SIZE)
            manager.Update()
            realtime_timer.Spin(STEP_SIZE)

            if system.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:
    print(f"simulation runtime error: {exc}")
    raise
finally:
    print("simulation completed or stopped; output streams flushed by context managers")
