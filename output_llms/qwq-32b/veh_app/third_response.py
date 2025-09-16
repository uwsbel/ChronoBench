import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.sensor as sens
import math
import os

# Set the data path for Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

print(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
initLoc = chrono.ChVectorD(0, -5, 0.4)
initRot = chrono.ChQuaternionD(1, 0, 0, 0)

# Visualization types
chassis_vis_type = veh.VisualizationType_MESH
suspension_vis_type = veh.VisualizationType_PRIMITIVES
steering_vis_type = veh.VisualizationType_PRIMITIVES
wheel_vis_type = veh.VisualizationType_NONE
tire_vis_type = veh.VisualizationType_MESH

trackPoint = chrono.ChVectorD(0.0, 0.0, 1.75)

step_size = 1e-3
tire_step_size = step_size

# Simulation end time
tend = 1000

render_step_size = 1.0 / 50
noise_model = "NONE"
update_rate = 10

image_width = 1280
image_height = 720
fov = 1.408
lag = 0
exposure_time = 0
vis = True

# Create and initialize the vehicle
gator = veh.Gator()
gator.SetContactMethod(chrono.ChContactMethod_NSC)
gator.SetChassisFixed(False)
gator.SetInitPosition(chrono.ChCoordsysD(initLoc, initRot))
gator.SetBrakeType(veh.BrakeType_SHAFTS)
gator.SetTireType(veh.TireModelType_TMEASY)
gator.SetTireStepSize(tire_step_size)
gator.SetInitFwdVel(0.0)
gator.Initialize()

gator.SetChassisVisualizationType(chassis_vis_type)
gator.SetSuspensionVisualizationType(suspension_vis_type)
gator.SetSteeringVisualizationType(steering_vis_type)
gator.SetWheelVisualizationType(wheel_vis_type)
gator.SetTireVisualizationType(tire_vis_type)

# Print vehicle info
print("Vehicle mass:   " + str(gator.GetVehicle().GetMass()))
print("Driveline type: " + gator.GetVehicle().GetDriveline().GetTemplateName())
print("Brake type:     " + gator.GetVehicle().GetBrake(1, veh.LEFT).GetTemplateName())
print("Tire type:      " + gator.GetVehicle().GetTire(1, veh.LEFT).GetTemplateName())
print("\n")

gator.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create terrain
terrain = veh.RigidTerrain(gator.GetSystem())
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 50, 50)
patch.SetColor(chrono.ChColor(0.8, 0.8, 1.0))
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 50, 50)
terrain.Initialize()

# Add box and cylinder
box = chrono.ChBodyEasyBox(1, 1, 1, 1000)
box.SetPos(chrono.ChVectorD(0, 0, 0.5))
box.SetFixed(True)
box.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
gator.GetSystem().Add(box)

cylinder = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.5, 1, 1000)
cylinder.SetPos(chrono.ChVectorD(0, 0, 1.5))
cylinder.SetFixed(True)
cylinder.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
gator.GetSystem().Add(cylinder)

# Create driver
driver = veh.ChDriver(gator.GetVehicle())
driver.Initialize()

# Sensor manager setup
manager = sens.ChSensorManager(gator.GetSystem())
manager.scene.AddPointLight(chrono.ChVectorF(2, 2.5, 100), chrono.ChColor(1, 1, 1), 500.0)

# Third person camera
cam_offset = chrono.ChFramed(chrono.ChVectorD(-8.0, 0, 1.45), chrono.Q_from_AngAxis(0.2, chrono.ChVectorD(0, 1, 0)))
cam = sens.ChCameraSensor(gator.GetChassisBody(), update_rate, cam_offset, image_width, image_height, fov)
cam.SetName("Third Person POV")
cam.PushFilter(sens.ChFilterVisualize(image_width, image_height, "Gator Camera"))
manager.AddSensor(cam)

# Lidar sensor
lidar_offset = chrono.ChFramed(chrono.ChVectorD(0.0, 0, 2), chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0)))
lidar = sens.ChLidarSensor(
    gator.GetChassisBody(),
    update_rate,
    lidar_offset,
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
    sens.LidarReturnMode_STRONGEST_RETURN
)
lidar.SetName("Lidar Sensor")
lidar.SetLag(lag)
lidar.SetCollectionWindow(1/update_rate)
lidar.PushFilter(sens.ChFilterDIAccess())
lidar.PushFilter(sens.ChFilterPCfromDepth())
lidar.PushFilter(sens.ChFilterXYZIAccess())
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))
manager.AddSensor(lidar)

# Depth Camera
depth_cam_offset = chrono.ChFramed(chrono.ChVectorD(-5.0, 0, 2), chrono.QUNIT)
depth_cam = sens.ChDepthCameraSensor(
    gator.GetChassisBody(),
    update_rate,
    depth_cam_offset,
    image_width,
    image_height,
    fov,
    30.0
)
depth_cam.SetName("Depth Camera")
depth_cam.PushFilter(sens.ChFilterVisualizeDepth(image_width, image_height, "Depth Map"))
manager.AddSensor(depth_cam)

# Logging setup
log_file = open("vehicle_state.log", "w")
log_file.write("Time,X,Y,Z,Yaw\n")

# Simulation loop
realtime_timer = chrono.ChRealtimeStepTimer()
time = 0
end_time = tend  # Corrected from original's end_time = 30

while time < end_time:
    time = gator.GetSystem().GetChTime()
    
    # Driver inputs
    driver.SetSteering(0.5)
    driver.SetThrottle(0.2)
    driver_inputs = driver.GetInputs()
    
    # Synchronize modules
    driver.Synchronize(time)
    terrain.Synchronize(time)
    gator.Synchronize(time, driver_inputs, terrain)
    
    manager.Update()
    
    # Advance simulation
    driver.Advance(step_size)
    terrain.Advance(step_size)
    gator.Advance(step_size)
    
    # Logging
    chassis = gator.GetChassisBody()
    pos = chassis.GetPos()
    rot = chassis.GetRot()
    _, _, yaw = rot.GetPitchRollYaw()
    log_file.write(f"{time},{pos.x},{pos.y},{pos.z},{yaw}\n")
    
    realtime_timer.Spin(step_size)

log_file.close()