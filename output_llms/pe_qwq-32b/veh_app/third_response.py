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
initLoc = chrono.ChVector3d(0, -5, 0.4)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
chassis_vis_type = veh.VisualizationType_MESH
suspension_vis_type = veh.VisualizationType_PRIMITIVES
steering_vis_type = veh.VisualizationType_PRIMITIVES
wheel_vis_type = veh.VisualizationType_NONE
tire_vis_type = veh.VisualizationType_MESH

# Point on the chassis tracked by the camera
trackPoint = chrono.ChVector3d(0.0, 0.0, 1.75)

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Simulation end time
tend = 1000

# Time interval between two render frames
render_step_size = 1.0 / 50  # FPS = 50

# Noise model for the sensors
noise_model = "NONE"  # No noise model

# Update rate in Hz for the sensors
update_rate = 10

# Image width and height for cameras
image_width = 1280
image_height = 720

# Camera's horizontal field of view
fov = 1.408

# Lag (in seconds) between sensing and when data becomes accessible
lag = 0

# Exposure time (in seconds) for each image
exposure_time = 0

# View camera images
vis = True

# Create the vehicle, set parameters, and initialize
gator = veh.Gator()
gator.SetContactMethod(chrono.ChContactMethod_NSC)
gator.SetChassisFixed(False)
gator.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
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

# Print vehicle information
print("Vehicle mass:   " + str(gator.GetVehicle().GetMass()))
print("Driveline type: " + gator.GetVehicle().GetDriveline().GetTemplateName())
print("Brake type:     " + gator.GetVehicle().GetBrake(1, veh.LEFT).GetTemplateName())
print("Tire type:      " + gator.GetVehicle().GetTire(1, veh.LEFT).GetTemplateName())
print("\n")

# Set collision system type
gator.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# ------------------
# Create the terrain
# ------------------
terrain = veh.RigidTerrain(gator.GetSystem())
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
# Corrected coordinatesys
patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysd(), 50, 50)
patch.SetColor(chrono.ChColor(0.8, 0.8, 1.0))
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 50, 50)
terrain.Initialize()

# Create box
box = chrono.ChBodyEasyBox(1, 1, 1, 1000)
box.SetPos(chrono.ChVector3d(0, 0, 0.5))
box.SetFixed(True)
box.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
gator.GetSystem().Add(box)  # Corrected Add() instead of AddBody()

# Create cylinder
cylinder = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.5, 1, 1000)
cylinder.SetPos(chrono.ChVector3d(0, 0, 1.5))
cylinder.SetFixed(True)
cylinder.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
gator.GetSystem().Add(cylinder)  # Corrected Add()

# Create the interactive driver system
driver = veh.ChDriver(gator.GetVehicle())
driver.Initialize()

# -----------------------
# Create a sensor manager
# -----------------------
manager = sens.ChSensorManager(gator.GetSystem())
intensity = 1.0
manager.scene.AddPointLight(chrono.ChVector3f(2, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)

# Existing camera
offset_pose = chrono.ChFramed(chrono.ChVector3d(-8.0, 0, 1.45), chrono.QuatFromAngleAxis(.2, chrono.ChVector3d(0, 1, 0)))
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

# Added Depth Camera
offset_pose_depth = chrono.ChFramed(
    chrono.ChVector3d(-5.0, 0, 2),
    chrono.QUNIT  # Identity rotation
)
depth_cam = sens.ChDepthCameraSensor(
    gator.GetChassisBody(),
    update_rate,
    offset_pose_depth,
    image_width,
    image_height,
    fov,
    30.0  # Max depth
)
depth_cam.SetName("Depth Camera")
depth_cam.PushFilter(sens.ChFilterVisualizeDepthMap(image_width, image_height, "Depth Map"))
manager.AddSensor(depth_cam)

# Existing Lidar setup remains unchanged

# Logging setup
logfile = open('vehicle_state.csv', 'w')
logfile.write("Time,X,Y,Z,Yaw\n")

# Simulation loop
realtime_timer = chrono.ChRealtimeStepTimer()
time = 0
end_time = 30
while time < end_time:
    time = gator.GetSystem().GetChTime()
    
    # Log vehicle state
    chassis = gator.GetChassisBody()
    pos = chassis.GetPos()
    frame = chassis.GetFrame_REF_to_abs()
    yaw = frame.Get_YawPitchRoll()[0]
    logfile.write(f"{time},{pos.x},{pos.y},{pos.z},{yaw}\n")

    # Driver inputs
    driver.SetSteering(0.5)
    driver.SetThrottle(0.2)
    driver_inputs = driver.GetInputs()

    # Update modules
    driver.Synchronize(time)
    terrain.Synchronize(time)
    gator.Synchronize(time, driver_inputs, terrain)
    manager.Update()

    # Advance simulation
    driver.Advance(step_size)
    terrain.Advance(step_size)
    gator.Advance(step_size)

    # Realtime synchronization
    realtime_timer.Spin(step_size)

# Close log file
logfile.close()