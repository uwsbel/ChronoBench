"""
PyChrono 9.0 HMMWV vehicle application with rigid terrain, fixed blue
obstacles, and a chassis-mounted lidar.  The NSC vehicle starts at y=-5,
drives with constant steering and throttle, and scans the box-and-cylinder
obstacle stack placed on the terrain centerline.
"""

import math

import pychrono.core as chrono
import pychrono.sensor as sens
import pychrono.vehicle as veh


# === Constants ===
STEP_SIZE = 2.0e-3
TIRE_STEP_SIZE = STEP_SIZE
SIM_END = 6.0
RENDER_FPS = 50.0
RENDER_STEP_SIZE = 1.0 / RENDER_FPS
RENDER_STEPS = max(1, math.ceil(RENDER_STEP_SIZE / STEP_SIZE))  # precomputed once

TERRAIN_LENGTH = 60.0
TERRAIN_WIDTH = 20.0
INIT_LOC = chrono.ChVector3d(0.0, -5.0, 0.4)
INIT_ROT = chrono.QUNIT
BOX_SIZE = chrono.ChVector3d(1.0, 1.0, 1.0)
BOX_POS = chrono.ChVector3d(0.0, 0.0, 0.5)
CYL_RADIUS = 0.5
CYL_HEIGHT = 1.0
CYL_POS = chrono.ChVector3d(0.0, 0.0, 1.5)
STEERING_INPUT = 0.5
THROTTLE_INPUT = 0.2
BRAKING_INPUT = 0.0

LIDAR_RATE = 5.0
LIDAR_H_SAMPLES = 800
LIDAR_V_CHANNELS = 300
LIDAR_H_FOV = 2.0 * chrono.CH_PI
LIDAR_MAX_V_FOV = chrono.CH_PI / 12.0
LIDAR_MIN_V_FOV = -chrono.CH_PI / 6.0
LIDAR_MAX_RANGE = 100.0
LIDAR_SAMPLE_RADIUS = 2
LIDAR_DIVERGENCE = 0.003
LIDAR_OFFSET = chrono.ChVector3d(0.0, 0.0, 2.0)


class ConstantVehicleDriver(veh.ChDriver):
    """Small scripted driver that exposes constant prompt-specified inputs."""

    def __init__(self, vehicle):
        super().__init__(vehicle)

    def Synchronize(self, time):
        self.SetSteering(STEERING_INPUT)
        self.SetThrottle(THROTTLE_INPUT)
        self.SetBraking(BRAKING_INPUT)


# === Vehicle & system ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
hmmwv.SetTireType(veh.TireModelType_RIGID)
hmmwv.SetTireStepSize(TIRE_STEP_SIZE)
hmmwv.Initialize()

system = hmmwv.GetSystem()  # cache: wrapper-owned ChSystemNSC reused by terrain and sensors
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetSolverType(chrono.ChSolver.Type_PSOR)
system.GetSolver().AsIterative().SetMaxIterations(150)
print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())

chassis = hmmwv.GetChassisBody()  # cache: lidar parent and log source
vehicle_model = hmmwv.GetVehicle()  # cache: wrapper vehicle for visualization and driver
# Wrapper-created components: system, chassis, axles, suspension links, steering links,
# wheels, tires, powertrain, and driveline are owned by veh.HMMWV_Full.

hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)


# === Terrain & obstacle bodies ===
terrain_mat = chrono.ChContactMaterialNSC()
terrain_mat.SetFriction(0.9)
terrain_mat.SetRestitution(0.01)

terrain = veh.RigidTerrain(system)
patch = terrain.AddPatch(terrain_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 40, 20)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.6))
terrain.Initialize()

obstacle_mat = chrono.ChContactMaterialNSC()
obstacle_mat.SetFriction(0.8)
obstacle_mat.SetRestitution(0.05)

blue_visual = chrono.ChVisualMaterial()
blue_visual.SetDiffuseColor(chrono.ChColor(0.05, 0.15, 0.95))
blue_visual.SetKdTexture(chrono.GetChronoDataFile("textures/blue.png"))

box = chrono.ChBodyEasyBox(BOX_SIZE.x, BOX_SIZE.y, BOX_SIZE.z, 1000.0, True, True, obstacle_mat)
box.SetName("blue_box_obstacle")
box.SetPos(BOX_POS)
box.SetFixed(True)
box.GetVisualShape(0).SetMaterial(0, blue_visual)
system.Add(box)

cylinder = chrono.ChBodyEasyCylinder(chrono.ChAxis_Z, CYL_RADIUS, CYL_HEIGHT, 1000.0, True, True, obstacle_mat)
cylinder.SetName("blue_cylinder_obstacle")
cylinder.SetPos(CYL_POS)
cylinder.SetFixed(True)
cylinder.GetVisualShape(0).SetMaterial(0, blue_visual)
system.Add(cylinder)


# === Lidar sensor ===
manager = sens.ChSensorManager(system)
lidar_offset_pose = chrono.ChFramed(
    LIDAR_OFFSET,
    chrono.QuatFromAngleAxis(0.0, chrono.ChVector3d(0.0, 1.0, 0.0)),
)
lidar = sens.ChLidarSensor(
    chassis,
    LIDAR_RATE,
    lidar_offset_pose,
    LIDAR_H_SAMPLES,
    LIDAR_V_CHANNELS,
    LIDAR_H_FOV,
    LIDAR_MAX_V_FOV,
    LIDAR_MIN_V_FOV,
    LIDAR_MAX_RANGE,
    sens.LidarBeamShape_RECTANGULAR,
    LIDAR_SAMPLE_RADIUS,
    LIDAR_DIVERGENCE,
    LIDAR_DIVERGENCE,
    sens.LidarReturnMode_STRONGEST_RETURN,
)
lidar.SetName("Chassis Lidar Sensor")
lidar.SetLag(0)
lidar.SetCollectionWindow(1.0 / LIDAR_RATE)
lidar.PushFilter(sens.ChFilterVisualize(LIDAR_H_SAMPLES, LIDAR_V_CHANNELS, "Lidar Depth and Intensity"))
lidar.PushFilter(sens.ChFilterDIAccess())
lidar.PushFilter(sens.ChFilterPCfromDepth())
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "XYZI Point Cloud"))
lidar.PushFilter(sens.ChFilterXYZIAccess())
manager.AddSensor(lidar)


# === Visualization & driver ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV with Lidar and Obstacles")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 8.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(vehicle_model)

driver = ConstantVehicleDriver(vehicle_model)
driver.Initialize()

realtime_timer = chrono.ChRealtimeStepTimer()
frame = 0
step_number = 0


# === Main loop ===
try:

    while vis.Run() and system.GetChTime() < SIM_END:
        sim_time = system.GetChTime()

        if step_number % RENDER_STEPS == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()
        driver.Synchronize(sim_time)
        driver_inputs = driver.GetInputs()
        driver_inputs.m_steering = STEERING_INPUT
        driver_inputs.m_throttle = THROTTLE_INPUT
        driver_inputs.m_braking = BRAKING_INPUT


        terrain.Synchronize(sim_time)
        hmmwv.Synchronize(sim_time, driver_inputs, terrain)
        vis.Synchronize(sim_time, driver_inputs)

        driver.Advance(STEP_SIZE)
        terrain.Advance(STEP_SIZE)
        hmmwv.Advance(STEP_SIZE)
        vis.Advance(STEP_SIZE)
        manager.Update()

        depth_buffer = lidar.GetMostRecentDIBuffer()
        if depth_buffer.HasData():
            pass

        step_number += 1
        realtime_timer.Spin(STEP_SIZE)
except (OSError, IOError) as exc:
    print(f"output I/O failed: {exc}")
    raise
except (RuntimeError, ValueError) as exc:
    print(f"simulation failed: {exc}")
    raise
finally:
    pass
