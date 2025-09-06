import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.sensor as sens
import math
import os


chrono.SetChronoDataPath(os.path.join(os.path.dirname(__file__), "..", "..", "..", "data"))
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


initLoc = chrono.ChVector3d(0, -5, 0.4)  
initRot = chrono.ChQuaterniond(1, 0, 0, 0)


chassis_vis_type = veh.VisualizationType_MESH
suspension_vis_type = veh.VisualizationType_PRIMITIVES
steering_vis_type = veh.VisualizationType_PRIMITIVES
wheel_vis_type = veh.VisualizationType_NONE
tire_vis_type = veh.VisualizationType_MESH


step_size = 1e-3
tire_step_size = step_size
render_step_size = 1.0 / 50


noise_model = "NONE"
update_rate = 10
image_width = 1280
image_height = 720
fov = 1.408


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


terrain = veh.RigidTerrain(gator.GetSystem())
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 50, 50)
patch.SetColor(chrono.ChColor(0.8, 0.8, 1.0))
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 50, 50)
terrain.Initialize()


driver = veh.ChDriver(gator.GetVehicle())
driver.Initialize()


manager = sens.ChSensorManager(gator.GetSystem())
intensity = 1.0
manager.scene.AddPointLight(chrono.ChVector3f(2, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)


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


lidar_offset = chrono.ChFramed(chrono.ChVector3d(0.0, 0, 2), chrono.ChQuaterniond(1, 0, 0, 0))
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
    sens.ChLidarSensor.BeamShape_RECTANGULAR,
    2,
    0.003,
    sens.ChLidarSensor.ReturnMode_STRONGEST_RETURN
)
lidar.SetName("Lidar Sensor")
lidar.PushFilter(sens.ChFilterD())
lidar.PushFilter(sens.ChFilterIntensity())
lidar.PushFilter(sens.ChFilterXYZI())
lidar.PushFilter(sens.ChFilterVisualize(640, 480, "Lidar Visualization"))
manager.AddSensor(lidar)


box = chrono.ChBodyEasyBox(1, 1, 1, 1000, True, True, patch_mat)
box.SetPos(chrono.ChVector3d(0, 0, 0.5))
box.SetName("Blue Box")
box_vis = box.GetVisualShape(0)
box_vis.SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
gator.GetSystem().Add(box)


cyl = chrono.ChBody()
cyl.SetPos(chrono.ChVector3d(0, 0, 1.5))
cyl.SetMass(10)
cyl.SetInertiaXX(chrono.ChVector3d(1, 1, 1))
cyl.EnableCollision(True)
cyl.SetCollide(True)


cyl_collision = chrono.ChCollisionShape(chrono.ChCollisionShape.Type_CYLINDER, patch_mat, 0.5, 1.0)
cyl.AddCollisionShape(cyl_collision)


cyl_vis = chrono.ChVisualShapeCylinder(0.5, 1.0)
cyl_vis.SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
cyl.AddVisualShape(cyl_vis)

cyl.SetName("Blue Cylinder")
gator.GetSystem().Add(cyl)


realtime_timer = chrono.ChRealtimeStepTimer()
time = 0
end_time = 30
while time < end_time:
    time = gator.GetSystem().GetChTime()
    
    
    driver.SetSteering(0.5)
    driver.SetThrottle(0.2)
    
    
    driver_inputs = driver.GetInputs()

    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    gator.Synchronize(time, driver_inputs, terrain)
    manager.Update()

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    gator.Advance(step_size)

    
    realtime_timer.Spin(step_size)