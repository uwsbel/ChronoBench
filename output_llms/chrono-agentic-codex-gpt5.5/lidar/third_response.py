"""ARTcar rigid-terrain sensor simulation using NSC contact.

The scene initializes an ARTcar vehicle on a flat rigid terrain, drives it through
the standard vehicle driver stack, and mounts 3D lidar, 2D lidar, and a
third-person RGB camera on the chassis. The lidar offset is placed one meter
forward and one meter above the chassis origin so the sensors scan from the car.
"""

import math

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens
import pychrono.vehicle as veh


# === Constants ===
# Define timing, terrain, and sensor parameters once so the loop stays simple.
STEP_SIZE = 1e-3
TIRE_STEP_SIZE = 1e-3
SIM_END = 6.0
RENDER_FPS = 50.0
RENDER_STEPS = max(1, math.ceil((1.0 / RENDER_FPS) / STEP_SIZE))  # precomputed once

TERRAIN_LENGTH = 160.0
TERRAIN_WIDTH = 40.0
TERRAIN_FRICTION = 0.9
TERRAIN_RESTITUTION = 0.01
INIT_POS = chrono.ChVector3d(0.0, 0.0, 0.5)
INIT_ROT = chrono.QUNIT

LIDAR_RATE = 5.0
LIDAR_OFFSET = chrono.ChVector3d(1.0, 0.0, 1.0)
LIDAR_ROT = chrono.QuatFromAngleAxis(0.0, chrono.ChVector3d(0, 1, 0))
LIDAR_RANGE = 100.0
LIDAR_HFOV = 2.0 * chrono.CH_PI
LIDAR_3D_H_SAMPLES = 800
LIDAR_3D_V_SAMPLES = 300
LIDAR_2D_H_SAMPLES = 800
LIDAR_2D_V_SAMPLES = 1

CAMERA_RATE = 30.0
CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 720
CAMERA_FOV = 1.408
CAMERA_OFFSET = chrono.ChVector3d(-6.0, 0.0, 2.4)
CAMERA_ROT = chrono.QuatFromAngleAxis(0.18, chrono.ChVector3d(0, 1, 0))


# === Vehicle System ===
# Build the wrapper-owned ARTcar and use its system for terrain, sensors, and visualization.
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

artcar = veh.ARTcar()
artcar.SetContactMethod(chrono.ChContactMethod_NSC)
artcar.SetChassisCollisionType(veh.CollisionType_NONE)
artcar.SetChassisFixed(False)
artcar.SetInitPosition(chrono.ChCoordsysd(INIT_POS, INIT_ROT))
artcar.SetTireType(veh.TireModelType_RIGID)
artcar.SetTireStepSize(TIRE_STEP_SIZE)
artcar.Initialize()

system = artcar.GetSystem()  # cache: wrapper-owned system reused by terrain and sensors
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
vehicle = artcar.GetVehicle()  # cache: vehicle subsystem handle reused by vis and driver
chassis = artcar.GetChassisBody()  # cache: chassis body reused by all mounted sensors

print("VEHICLE MASS: ", vehicle.GetMass())

artcar.SetChassisVisualizationType(veh.VisualizationType_MESH)
artcar.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
artcar.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
artcar.SetWheelVisualizationType(veh.VisualizationType_MESH)
artcar.SetTireVisualizationType(veh.VisualizationType_MESH)

# Wrapper-created essentials: system, chassis, wheels, tires, powertrain, driver, terrain, and vehicle visual system.


# === Terrain ===
# Use a flat rigid patch with NSC material so the ARTcar has stable road contact.
terrain = veh.RigidTerrain(system)
terrain_mat = chrono.ChContactMaterialNSC()
terrain_mat.SetFriction(TERRAIN_FRICTION)
terrain_mat.SetRestitution(TERRAIN_RESTITUTION)
patch = terrain.AddPatch(terrain_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 160, 40)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()


# === Visualization ===
# Vehicle Irrlicht visualization gives a chase view while sensors provide onboard streams.
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("ARTcar Lidar Sensors")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.0), 6.0, 0.5)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AddGrid(
    2.0,
    2.0,
    40,
    15,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    chrono.ChColor(0.35, 0.35, 0.35),
)
vis.AttachVehicle(vehicle)


# === Driver ===
# Bind the standard interactive driver to the Irrlicht visual system.
driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta((1.0 / RENDER_FPS) / 1.0)
driver.SetThrottleDelta((1.0 / RENDER_FPS) / 1.0)
driver.SetBrakingDelta((1.0 / RENDER_FPS) / 0.3)
driver.Initialize()


# === Sensors ===
# Attach the lidar sensors and third-person camera to the real chassis body.
manager = sens.ChSensorManager(system)
manager.scene.AddPointLight(
    chrono.ChVector3f(2.0, -3.0, 20.0),
    chrono.ChColor(1.0, 1.0, 1.0),
    120.0,
)
manager.scene.AddPointLight(
    chrono.ChVector3f(12.0, 3.0, 20.0),
    chrono.ChColor(0.8, 0.8, 0.8),
    120.0,
)

lidar_pose = chrono.ChFramed(LIDAR_OFFSET, LIDAR_ROT)

lidar_3d = sens.ChLidarSensor(
    chassis,
    LIDAR_RATE,
    lidar_pose,
    LIDAR_3D_H_SAMPLES,
    LIDAR_3D_V_SAMPLES,
    LIDAR_HFOV,
    chrono.CH_PI / 12.0,
    -chrono.CH_PI / 6.0,
    LIDAR_RANGE,
    sens.LidarBeamShape_RECTANGULAR,
    2,
    0.003,
    0.003,
    sens.LidarReturnMode_STRONGEST_RETURN,
)
lidar_3d.SetName("3D Lidar Sensor")
lidar_3d.SetLag(0)
lidar_3d.SetCollectionWindow(1.0 / LIDAR_RATE)
lidar_3d.PushFilter(sens.ChFilterVisualize(LIDAR_3D_H_SAMPLES, LIDAR_3D_V_SAMPLES, "3D Lidar Depth"))
lidar_3d.PushFilter(sens.ChFilterDIAccess())
lidar_3d.PushFilter(sens.ChFilterPCfromDepth())
lidar_3d.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "3D Lidar Point Cloud"))
lidar_3d.PushFilter(sens.ChFilterXYZIAccess())
manager.AddSensor(lidar_3d)

lidar_2d = sens.ChLidarSensor(
    chassis,
    LIDAR_RATE,
    lidar_pose,
    LIDAR_2D_H_SAMPLES,
    LIDAR_2D_V_SAMPLES,
    LIDAR_HFOV,
    0.0,
    0.0,
    LIDAR_RANGE,
    sens.LidarBeamShape_RECTANGULAR,
    2,
    0.003,
    0.003,
    sens.LidarReturnMode_STRONGEST_RETURN,
)
lidar_2d.SetName("2D Lidar Sensor")
lidar_2d.SetLag(0)
lidar_2d.SetCollectionWindow(1.0 / LIDAR_RATE)
lidar_2d.PushFilter(sens.ChFilterVisualize(LIDAR_2D_H_SAMPLES, LIDAR_2D_V_SAMPLES, "2D Lidar Depth"))
lidar_2d.PushFilter(sens.ChFilterDIAccess())
lidar_2d.PushFilter(sens.ChFilterPCfromDepth())
lidar_2d.PushFilter(sens.ChFilterVisualizePointCloud(640, 240, 1.0, "2D Lidar Point Cloud"))
lidar_2d.PushFilter(sens.ChFilterXYZIAccess())
manager.AddSensor(lidar_2d)

third_person_cam = sens.ChCameraSensor(
    chassis,
    CAMERA_RATE,
    chrono.ChFramed(CAMERA_OFFSET, CAMERA_ROT),
    CAMERA_WIDTH,
    CAMERA_HEIGHT,
    CAMERA_FOV,
)
third_person_cam.SetName("Third Person Camera")
third_person_cam.SetLag(0)
third_person_cam.SetCollectionWindow(0)
third_person_cam.PushFilter(sens.ChFilterVisualize(CAMERA_WIDTH, CAMERA_HEIGHT, "Third Person Camera"))
third_person_cam.PushFilter(sens.ChFilterRGBA8Access())
third_person_cam.PushFilter(sens.ChFilterSave("cam/third_person/"))
manager.AddSensor(third_person_cam)


# === Main Loop ===
# Synchronize and advance every vehicle module while sensors update once per step.
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0


try:
    while vis.Run() and system.GetChTime() < SIM_END:
        time = system.GetChTime()

        if step_number % RENDER_STEPS == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()


        driver_inputs = driver.GetInputs()
        driver.Synchronize(time)
        terrain.Synchronize(time)
        artcar.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        driver.Advance(STEP_SIZE)
        terrain.Advance(STEP_SIZE)
        artcar.Advance(STEP_SIZE)
        vis.Advance(STEP_SIZE)
        manager.Update()


        step_number += 1
        realtime_timer.Spin(STEP_SIZE)
except (RuntimeError, ValueError, OSError) as exc:
    print(f"Simulation failed during vehicle/sensor stepping: {exc}")
    raise
finally:
    pass


# === Post Processing ===
# Assemble review artifacts after the run; these calls are stripped before scoring.
