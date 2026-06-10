"""
Lidar simulation with ARTcar vehicle, rigid terrain, and third-person camera.

System: NSC (ChContactMethod_NSC)
Bodies: ARTcar chassis + 4 wheels, rigid terrain patch, sensing box
Sensors: 3D lidar + 2D lidar attached to chassis, third-person camera attached to chassis
Expected behavior: Vehicle drives with lidars scanning; camera renders third-person view.
"""

import os
import math
import csv
import pychrono.core as chrono
import pychrono.sensor as sens
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Parameters ===
step_size = 1e-3
sim_end = 40.0
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * step_size)))

# Lidar parameters
noise_model = "NONE"
return_mode = sens.LidarReturnMode_STRONGEST_RETURN
update_rate = 5.0
horizontal_samples = 800
vertical_samples = 300
horizontal_fov = 2 * chrono.CH_PI
max_vert_angle = chrono.CH_PI / 12
min_vert_angle = -chrono.CH_PI / 6
lag = 0
collection_time = 1.0 / update_rate
sample_radius = 2
divergence_angle = 0.003

# ARTcar init position
initLoc = chrono.ChVector3d(0, -5.0, 0.5)
initRot = chrono.QuatFromAngleZ(1.57)

# === Data paths ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# === Create ARTcar vehicle ===
car = veh.ARTcar()
car.SetContactMethod(chrono.ChContactMethod_NSC)
car.SetChassisFixed(False)
car.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
car.SetTireType(veh.TireModelType_TMEASY)
car.SetTireStepSize(step_size)
car.SetMaxMotorVoltageRatio(0.12)
car.SetStallTorque(0.3)
car.SetTireRollingResistance(0.06)
car.Initialize()

car.SetChassisVisualizationType(veh.VisualizationType_MESH)
car.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
car.SetSteeringVisualizationType(veh.VisualizationType_MESH)
car.SetWheelVisualizationType(veh.VisualizationType_MESH)
car.SetTireVisualizationType(veh.VisualizationType_MESH)

system = car.GetSystem()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
print("VEHICLE MASS: ", car.GetVehicle().GetMass())

# === Driver ===
driver = veh.ChDriver(car.GetVehicle())
driver.Initialize()

# === Terrain ===
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(system)
patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    20, 20
)
patch.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Sensing box (lidar target) ===
side = 4
box = chrono.ChBodyEasyBox(side, side, side, 1000)
box.SetPos(chrono.ChVector3d(0, 0, 0))
box.GetVisualModel().GetShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
box.SetFixed(True)
system.Add(box)

# === Sensor manager ===
manager = sens.ChSensorManager(system)

# === 3D Lidar attached to chassis ===
lidar_offset = chrono.ChFramed(
    chrono.ChVector3d(1.0, 0, 1),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
)
lidar = sens.ChLidarSensor(
    car.GetChassisBody(),
    update_rate,
    lidar_offset,
    horizontal_samples,
    vertical_samples,
    horizontal_fov,
    max_vert_angle,
    min_vert_angle,
    100.0,
    sens.LidarBeamShape_RECTANGULAR,
    sample_radius,
    divergence_angle,
    divergence_angle,
    return_mode
)
lidar.SetName("Lidar Sensor")
lidar.SetLag(lag)
lidar.SetCollectionWindow(collection_time)

if noise_model == "CONST_NORMAL_XYZI":
    lidar.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))

lidar.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Raw Lidar Depth Data"))
lidar.PushFilter(sens.ChFilterDIAccess())
lidar.PushFilter(sens.ChFilterPCfromDepth())
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))
lidar.PushFilter(sens.ChFilterXYZIAccess())
manager.AddSensor(lidar)

# === 2D Lidar attached to chassis ===
lidar_2d = sens.ChLidarSensor(
    car.GetChassisBody(),
    update_rate,
    lidar_offset,
    horizontal_samples,
    1,
    horizontal_fov,
    0.0,
    0.0,
    100.0,
    sens.LidarBeamShape_RECTANGULAR,
    sample_radius,
    divergence_angle,
    divergence_angle,
    return_mode
)
lidar_2d.SetName("2D Lidar Sensor")
lidar_2d.SetLag(lag)
lidar_2d.SetCollectionWindow(collection_time)

if noise_model == "CONST_NORMAL_XYZI":
    lidar_2d.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))

lidar_2d.PushFilter(sens.ChFilterVisualize(horizontal_samples, 1, "Raw 2D Lidar Depth Data"))
lidar_2d.PushFilter(sens.ChFilterDIAccess())
lidar_2d.PushFilter(sens.ChFilterPCfromDepth())
lidar_2d.PushFilter(sens.ChFilterXYZIAccess())
manager.AddSensor(lidar_2d)

# === Third-person camera attached to chassis ===
cam_offset = chrono.ChFramed(
    chrono.ChVector3d(-2.0, 0, 1),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
)
camera = sens.ChCameraSensor(
    car.GetChassisBody(),
    update_rate,
    cam_offset,
    horizontal_samples,
    vertical_samples,
    1.48,
)
camera.SetName("Third Person Camera")
camera.SetLag(0)
camera.SetCollectionWindow(0)
camera.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "third person view"))
manager.AddSensor(camera)

# === Irrlicht Visualization ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("ARTcar Lidar Simulation")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AddGrid(0.5, 0.5, 40, 40, chrono.ChCoordsysd(), chrono.ChColor(0.4, 0.4, 0.4))
vis.AttachVehicle(car.GetVehicle())

# === Review-only: frame capture setup ===

# === Main loop ===
ch_time = 0.0
frame = 0

while vis.Run() and ch_time < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()


    for _ in range(render_every):
        manager.Update()

        # Scripted driver inputs (zero throttle/steering for stationary demo)
        driver.SetSteering(0.0)
        driver.SetThrottle(0.0)
        driver_inputs = driver.GetInputs()

        driver.Synchronize(ch_time)
        terrain.Synchronize(ch_time)
        car.Synchronize(ch_time, driver_inputs, terrain)
        vis.Synchronize(ch_time, driver_inputs)

        driver.Advance(step_size)
        terrain.Advance(step_size)
        car.Advance(step_size)

        ch_time = system.GetChTime()


# === Review-only: CSV logging + video assembly ===
