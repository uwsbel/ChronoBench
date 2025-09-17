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

# Visualization types for vehicle parts
chassis_vis_type = veh.VisualizationType_MESH
suspension_vis_type = veh.VisualizationType_PRIMITIVES
steering_vis_type = veh.VisualizationType_PRIMITIVES
wheel_vis_type = veh.VisualizationType_NONE
tire_vis_type = veh.VisualizationType_MESH

# Point on the chassis tracked by the camera
trackPoint = chrono.ChVectorD(0.0, 0.0, 1.75)

# Simulation parameters
step_size = 1e-3
tire_step_size = step_size
tend = 1000
render_step_size = 1.0 / 50  # FPS = 50

# Sensor parameters
noise_model = "NONE"
update_rate = 10
image_width = 1280
image_height = 720
fov = 1.408
lag = 0
exposure_time = 0
vis = True

# Create the vehicle
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

# Set collision system type
gator.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create terrain
terrain = veh.RigidTerrain(gator.GetSystem())
material = terrain.GetMaterial(0)
material.SetFriction(0.9)
material.SetRestitution(0.01)
patch = terrain.AddPatch(0, chrono.CSYSNORM, 50, 50)
patch.SetColor(chrono.ChColor(0.8, 0.8, 1.0))
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 50, 50)
terrain.Initialize()

# Add box and cylinder objects
box = chrono.ChBodyEasyBox(1, 1, 1, 1000)  # 1x1x1 box, density 1000 kg/m³
box.SetPos(chrono.ChVectorD(0, 0, 0.5))
box.SetMaterialSurface(material)
box.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"), 1, 1)
gator.GetSystem().Add(box)

cylinder = chrono.ChBodyEasyCylinder(0.5, 1, 1000)  # radius 0.5, height 1
cylinder.SetPos(chrono.ChVectorD(0, 0, 1.5))
cylinder.SetMaterialSurface(material)
cylinder.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"), 1, 1)
gator.GetSystem().Add(cylinder)

# Create driver
driver = veh.ChDriver(gator.GetVehicle())
driver.Initialize()

# Create sensor manager
manager = sens.ChSensorManager(gator.GetSystem())
intensity = 1.0
manager.scene.AddPointLight(chrono.ChVectorF(2, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)

# Camera sensor
offset_pose = chrono.ChFrameD(chrono.ChVectorD(-8.0, 0, 1.45), chrono.Q_from_AngAxis(0.2, chrono.ChVectorD(0, 1, 0)))
cam = sens.ChCameraSensor(
    gator.GetChassisBody(),
    update_rate,
    offset_pose,
    image_width,
    image_height,
    fov
)
cam.SetName("Third Person POV")
cam.PushFilter(sens.ChFilterVisualize(image_width, image_height, "Gator Camera"))
manager.AddSensor(cam)

# Lidar sensor
lidar_pose = chrono.ChFrameD(chrono.ChVectorD(0.0, 0.0, 2.0), chrono.QUNIT)
lidar = sens.ChLidarSensor(
    gator.GetChassisBody(),
    update_rate,
    lidar_pose,
    800,   # Horizontal samples
    300,   # Vertical channels
    2 * chrono.CH_C_PI,  # 360 degrees horizontal FOV
    chrono.CH_C_PI / 12,  # Max vertical angle
    -chrono.CH_C_PI / 6,  # Min vertical angle
    100.0  # Max range
)
lidar.SetBeamShape(sens.LidarBeamShape_RECTANGULAR)
lidar.SetSampleRadius(2)
lidar.SetDivergenceAngle(0.003)
lidar.SetReturnMode(sens.LidarReturnMode_STRONGEST)
lidar.PushFilter(sens.ChFilterDepth())
lidar.PushFilter(sens.ChFilterIntensity())
lidar.PushFilter(sens.ChFilterXYZI())
lidar.PushFilter(sens.ChFilterVisualizePointCloud("Lidar Visualization"))
manager.AddSensor(lidar)

# Simulation loop
realtime_timer = chrono.ChRealtimeStepTimer()
end_time = 30  # As per original loop's end_time variable
while gator.GetSystem().GetChTime() < end_time:
    time = gator.GetSystem().GetChTime()
    
    # Driver inputs
    driver.SetSteering(0.5)
    driver.SetThrottle(0.2)
    driver_inputs = driver.GetInputs()
    
    # Synchronize and advance modules
    driver.Synchronize(time)
    terrain.Synchronize(time)
    gator.Synchronize(time, driver_inputs, terrain)
    
    manager.Update()
    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    gator.Advance(step_size)
    
    realtime_timer.Spin(step_size)