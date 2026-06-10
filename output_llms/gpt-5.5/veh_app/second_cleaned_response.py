import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens
import os


veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

print(chrono.GetChronoDataPath() + "vehicle/")


initLoc = chrono.ChVector3d(0, -5, 0.4)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)


chassis_vis_type = veh.VisualizationType_MESH
suspension_vis_type = veh.VisualizationType_PRIMITIVES
steering_vis_type = veh.VisualizationType_PRIMITIVES
wheel_vis_type = veh.VisualizationType_NONE
tire_vis_type = veh.VisualizationType_MESH


trackPoint = chrono.ChVector3d(0.0, 0.0, 1.75)


step_size = 1e-3
tire_step_size = step_size


end_time = 30


render_step_size = 1.0 / 50  


noise_model = "NONE"


update_rate = 10


image_width = 1280
image_height = 720


fov = 1.408


lag = 0
exposure_time = 0


vis = True




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


gator.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


print("Vehicle mass:   " + str(gator.GetVehicle().GetMass()))
print("Driveline type: " + gator.GetVehicle().GetDriveline().GetTemplateName())
print("Brake type:     " + gator.GetVehicle().GetBrake(1, veh.LEFT).GetTemplateName())
print("Tire type:      " + gator.GetVehicle().GetTire(1, veh.LEFT).GetTemplateName())
print("\n")




terrain = veh.RigidTerrain(gator.GetSystem())

patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 50, 50)
patch.SetColor(chrono.ChColor(0.8, 0.8, 1.0))
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 50, 50)

terrain.Initialize()




object_mat = chrono.ChContactMaterialNSC()
object_mat.SetFriction(0.8)
object_mat.SetRestitution(0.01)

blue_color = chrono.ChColor(0.0, 0.0, 1.0)
blue_texture = chrono.GetChronoDataFile("textures/blue.png")


box = chrono.ChBodyEasyBox(1.0, 1.0, 1.0, 1000.0, True, True, object_mat)
box.SetPos(chrono.ChVector3d(0.0, 0.0, 0.5))
box.SetFixed(True)
box.GetVisualShape(0).SetColor(blue_color)
if os.path.isfile(blue_texture):
    box.GetVisualShape(0).SetTexture(blue_texture)
gator.GetSystem().Add(box)


cylinder = chrono.ChBodyEasyCylinder(
    chrono.ChAxis_Z,
    0.5,
    1.0,
    1000.0,
    True,
    True,
    object_mat,
)
cylinder.SetPos(chrono.ChVector3d(0.0, 0.0, 1.5))
cylinder.SetFixed(True)
cylinder.GetVisualShape(0).SetColor(blue_color)
if os.path.isfile(blue_texture):
    cylinder.GetVisualShape(0).SetTexture(blue_texture)
gator.GetSystem().Add(cylinder)




driver = veh.ChDriver(gator.GetVehicle())
driver.Initialize()




manager = sens.ChSensorManager(gator.GetSystem())

intensity = 1.0
manager.scene.AddPointLight(
    chrono.ChVector3f(2, 2.5, 100),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)




camera_offset_pose = chrono.ChFramed(
    chrono.ChVector3d(-8.0, 0.0, 1.45),
    chrono.QuatFromAngleAxis(0.2, chrono.ChVector3d(0, 1, 0)),
)

cam = sens.ChCameraSensor(
    gator.GetChassisBody(),
    update_rate,
    camera_offset_pose,
    image_width,
    image_height,
    fov,
)
cam.SetName("Third Person POV")

if vis:
    cam.PushFilter(sens.ChFilterVisualize(image_width, image_height, "Gator Camera"))

manager.AddSensor(cam)




lidar_horizontal_samples = 800
lidar_vertical_channels = 300
lidar_horizontal_fov = 2 * chrono.CH_PI
lidar_max_vert_angle = chrono.CH_PI / 12
lidar_min_vert_angle = -chrono.CH_PI / 6
lidar_max_range = 100.0
lidar_sample_radius = 2
lidar_divergence_angle = 0.003

lidar_offset_pose = chrono.ChFramed(
    chrono.ChVector3d(0.0, 0.0, 2.0),
    chrono.ChQuaterniond(1, 0, 0, 0),
)

lidar = sens.ChLidarSensor(
    gator.GetChassisBody(),
    update_rate,
    lidar_offset_pose,
    lidar_horizontal_samples,
    lidar_vertical_channels,
    lidar_horizontal_fov,
    lidar_max_vert_angle,
    lidar_min_vert_angle,
    lidar_max_range,
    sens.LidarBeamShape_RECTANGULAR,
    lidar_sample_radius,
    lidar_divergence_angle,
    sens.LidarReturnMode_STRONGEST_RETURN,
)
lidar.SetName("Gator Lidar")


lidar.PushFilter(sens.ChFilterDIAccess())


if vis:
    lidar.PushFilter(
        sens.ChFilterVisualize(
            lidar_horizontal_samples,
            lidar_vertical_channels,
            "Lidar Depth/Intensity",
        )
    )


lidar.PushFilter(sens.ChFilterPCfromDepth())


lidar.PushFilter(sens.ChFilterXYZIAccess())


if vis:
    lidar.PushFilter(
        sens.ChFilterVisualizePointCloud(
            1280,
            720,
            1.0,
            "Lidar XYZI Point Cloud",
        )
    )

manager.AddSensor(lidar)




realtime_timer = chrono.ChRealtimeStepTimer()

time = 0.0
while time < end_time:
    time = gator.GetSystem().GetChTime()

    
    driver.SetSteering(0.5)
    driver.SetThrottle(0.2)
    driver.SetBraking(0.0)

    
    driver_inputs = driver.GetInputs()

    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    gator.Synchronize(time, driver_inputs, terrain)

    
    manager.Update()

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    gator.Advance(step_size)

    
    realtime_timer.Spin(step_size)