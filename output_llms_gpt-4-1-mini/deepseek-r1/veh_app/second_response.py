import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.sensor as sens
import math
import os

# Set the data path for Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

print(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation (MODIFIED)
initLoc = chrono.ChVector3d(0, -5, 0.4)  # Changed y-coordinate to -5
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts
chassis_vis_type = veh.VisualizationType_MESH
suspension_vis_type = veh.VisualizationType_PRIMITIVES
steering_vis_type = veh.VisualizationType_PRIMITIVES
wheel_vis_type = veh.VisualizationType_NONE
tire_vis_type = veh.VisualizationType_MESH

# Point on the chassis tracked by the camera
trackPoint = chrono.ChVector3d(0.0, 0.0, 1.75)

# Simulation parameters
step_size = 1e-3
tire_step_size = step_size
tend = 1000
render_step_size = 1.0 / 50  # 50 FPS

# Sensor parameters
noise_model = "NONE"
update_rate = 10
image_width = 1280
image_height = 720
fov = 1.408
lag = 0
exposure_time = 0
vis = True

# Create and initialize the Gator vehicle
gator = veh.Gator()
gator.SetContactMethod(chrono.ChContactMethod_NSC)
gator.SetChassisFixed(False)
gator.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
gator.SetBrakeType(veh.BrakeType_SHAFTS)
gator.SetTireType(veh.TireModelType_TMEASY)
gator.SetTireStepSize(tire_step_size)
gator.SetInitFwdVel(0.0)
gator.Initialize()

# Set visualization modes
gator.SetChassisVisualizationType(chassis_vis_type)
gator.SetSuspensionVisualizationType(suspension_vis_type)
gator.SetSteeringVisualizationType(steering_vis_type)
gator.SetWheelVisualizationType(wheel_vis_type)
gator.SetTireVisualizationType(tire_vis_type)

# Print vehicle information
print("Vehicle mass:   " + str(gator.GetVehicle().GetMass()))
print("Driveline type: " + gator.GetVehicle().GetDriveline().GetTemplateName())
print("Brake type:     " + gator.GetVehicle().GetBrake(1, veh.LEFT).GetTemplateName())
print("Tire type:      " + gator.GetVehicle().GetTire(1, veh.LEFT).GetTemplateName())
print("\n")

# Set collision system type
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

# Add blue box (NEW)
box_body = chrono.ChBody()
box_body.SetPos(chrono.ChVector3d(0, 0, 0.5))
box_body.SetFixed(True)
box_shape = chrono.ChBoxShape(1, 1, 1)
box_shape.SetColor(chrono.ChColor(0.0, 0.0, 1.0))  # Blue color
box_body.AddVisualShape(box_shape)
gator.GetSystem().AddBody(box_body)

# Add blue cylinder (NEW)
cylinder_body = chrono.ChBody()
cylinder_body.SetPos(chrono.ChVector3d(0, 0, 1.5))
cylinder_body.SetFixed(True)
cylinder_shape = chrono.ChCylinderShape(0.5, 1)  # Radius, height
cylinder_shape.SetColor(chrono.ChColor(0.0, 0.0, 1.0))  # Blue color
cylinder_body.AddVisualShape(cylinder_shape)
gator.GetSystem().AddBody(cylinder_body)

# Create driver system
driver = veh.ChDriver(gator.GetVehicle())
driver.Initialize()

# Create sensor manager (MODIFIED)
manager = sens.ChSensorManager(gator.GetSystem())
intensity = 1.0
manager.scene.AddPointLight(chrono.ChVector3f(2, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)

# Create camera (FIXED frame initialization)
offset_pose = chrono.ChFrameD(chrono.ChVector3d(-8.0, 0, 1.45), 
                            chrono.Q_from_AngAxis(0.2, chrono.ChVector3d(0, 1, 0)))  # Fixed quaternion constructor
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

# Add Lidar sensor (NEW)
lidar_offset = chrono.ChFrameD(chrono.ChVector3d(0.0, 0, 2), chrono.QUNIT)
lidar = sens.ChLidarSensor(
    gator.GetChassisBody(),
    update_rate,
    lidar_offset,
    800,  # horizontal samples
    300,  # vertical channels
    2 * chrono.CH_PI,  # horizontal FOV
    chrono.CH_PI / 12,  # vertical max angle
    -chrono.CH_PI / 6,  # vertical min angle
    100.0  # max distance
)
lidar.SetName("Lidar Sensor")
lidar.SetBeamShape(sens.BeamShape_RECTANGULAR)
lidar.SetSampleRadius(2)
lidar.SetDivergenceAngle(0.003)
lidar.SetReturnMode(sens.ReturnMode_STRONGEST_RETURN)
lidar.PushFilter(sens.ChFilterDIAccess())
lidar.PushFilter(sens.ChFilterXYZIAccess())
lidar.PushFilter(sens.ChFilterVisualize(800, 300, "Lidar Point Cloud"))
manager.AddSensor(lidar)

# Simulation loop
realtime_timer = chrono.ChRealtimeStepTimer()
time = 0
end_time = 30
while time < end_time:
    time = gator.GetSystem().GetChTime()
    
    # Set driver inputs (MODIFIED values already in original)
    driver.SetSteering(0.5)
    driver.SetThrottle(0.2)

    # Update modules
    driver.Synchronize(time)
    terrain.Synchronize(time)
    gator.Synchronize(time, driver.GetInputs(), terrain)
    manager.Update()

    # Advance simulation
    driver.Advance(step_size)
    terrain.Advance(step_size)
    gator.Advance(step_size)
    realtime_timer.Spin(step_size)