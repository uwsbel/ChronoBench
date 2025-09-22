import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.sensor as sens
import numpy as np
import math

# ------------------------------------------------------------------------
# 1) SETUP CHRONO DATA PATH
# ------------------------------------------------------------------------
# Make sure CHRONO_DATA_DIR is set in your environment, or hard‐code it here:
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

# ------------------------------------------------------------------------
# 2) PARAMETERS
# ------------------------------------------------------------------------
initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

vis_type = veh.VisualizationType_MESH
chassis_coll = veh.CollisionType_NONE
tire_model  = veh.TireModelType_TMEASY

terrainLength = 100.0
terrainWidth  = 100.0

contact_method = chrono.ChContactMethod_NSC
step_size       = 1e-3
render_fps      = 50
render_step     = 1.0 / render_fps

trackPoint = chrono.ChVector3d(-3, 0, 1.1)

# ------------------------------------------------------------------------
# 3) CREATE VEHICLE
# ------------------------------------------------------------------------
vehicle = veh.MAN_10t()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_coll)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(step_size)
vehicle.Initialize()

# Visualization
vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

# Use Bullet for speed
vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

print("VEHICLE MASS:", vehicle.GetMass())

# ------------------------------------------------------------------------
# 4) CREATE TERRAIN
# ------------------------------------------------------------------------
# Use NSC material
patch_mat = chrono.ChMaterialSurfaceNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat,
                         chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0),
                                            chrono.ChQuaterniond(1, 0, 0, 0)),
                         terrainLength, terrainWidth)

# Change texture to grass.jpg
patch.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.4, 0.7, 0.4))
terrain.Initialize()

# ------------------------------------------------------------------------
# 5) ADD RANDOM BOXES
# ------------------------------------------------------------------------
# Scatter some boxes on the terrain
num_boxes = 10
for i in range(num_boxes):
    # random size between 0.3 and 1.0 m
    sx = np.random.uniform(0.3, 1.0)
    sy = np.random.uniform(0.3, 1.0)
    sz = np.random.uniform(0.3, 1.0)
    # create a body-easy box (has built-in collision & visualization)
    box = chrono.ChBodyEasyBox(sx, sy, sz,
                               2000,     # density
                               True,     # visualization
                               True)     # collision
    # random position
    x = np.random.uniform(-terrainLength/2 + sx, terrainLength/2 - sx)
    y = np.random.uniform(-terrainWidth/2  + sy, terrainWidth/2  - sy)
    box.SetPos(chrono.ChVectorD(x, y, 2.0 + sz/2))
    box.SetBodyFixed(False)
    vehicle.GetSystem().Add(box)

# ------------------------------------------------------------------------
# 6) IRRLICHT VISUALIZATION
# ------------------------------------------------------------------------
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("MAN_10t + LiDAR + Boxes")
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 15.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle)

# ------------------------------------------------------------------------
# 7) DRIVER (INTERACTIVE)
# ------------------------------------------------------------------------
driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(render_step / 1.0)
driver.SetThrottleDelta(render_step / 1.0)
driver.SetBrakingDelta(render_step / 0.3)
driver.Initialize()

# ------------------------------------------------------------------------
# 8) SENSOR MANAGER & LIDAR
# ------------------------------------------------------------------------
sensor_manager = sens.ChSensorManager(vehicle.GetSystem())
sensor_manager.SetVerbose(False)
sensor_manager.SetRenderRate(20)    # Render sensor data at 20 Hz

# Create a 3D LiDAR attached to the chassis
lidar = sens.ChLidarSensor(
    parent=vehicle.GetChassis(),       # attach to chassis body
    updateRate=10.0,                   # Hz
    pointRate=100000,                  # points per second
    horizontalSamples=512,
    verticalSamples=32,
    horizontalFOV=chrono.CH_C_PI,      # 180°
    verticalFOV=chrono.CH_C_PI / 6,    # 30°
    maxDistance=100.0,
    lag=chrono.ChFrame<double>(),      # no lag
)
# Position the lidar on the roof
lidar_pose = chrono.ChFrameD(
    chrono.ChVectorD(0.0, 0.0, 1.8), 
    chrono.ChQuaterniond(1, 0, 0, 0)
)
lidar.SetFrame(lidar_pose)
sensor_manager.AddSensor(lidar)

# ------------------------------------------------------------------------
# 9) SIMULATION LOOP
# ------------------------------------------------------------------------
realtime_timer = chrono.ChRealtimeStepTimer()
step_number    = 0
render_steps   = math.ceil(render_step / step_size)

while vis.Run():
    t = vehicle.GetSystem().GetChTime()

    # Render 3D scene
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    # Drive inputs
    inputs = driver.GetInputs()

    # SYNCHRONIZE all modules
    driver.Synchronize(t)
    terrain.Synchronize(t)
    vehicle.Synchronize(t, inputs, terrain)
    vis.Synchronize(t, inputs)

    # Update & render sensors
    sensor_manager.Update()      # poll new data if ready
    sensor_manager.Render()      # draw onto off‐screen buffers
    sensor_manager.PostProcess()

    # ADVANCE all modules by one timestep
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    step_number += 1
    realtime_timer.Spin(step_size)