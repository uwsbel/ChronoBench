import os
import math
import pychrono as chrono
import pychrono.robot as robot
import pychrono.sensor as sens
from pychrono import irrlicht as chronoirr






system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))

chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)






ground_mat = chrono.ChContactMaterialNSC()
ground_mat.SetFriction(0.9)

ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, -0.5))
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)


box = chrono.ChBodyEasyBox(0.25, 5, 0.25, 1000, True, True, ground_mat)
box.SetPos(chrono.ChVector3d(0, 0, 0.125))
box.SetFixed(True)
box.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
system.Add(box)






rover = robot.Curiosity(system)

driver = robot.CuriosityDCMotorControl()
rover.SetDriver(driver)


init_pos = chrono.ChVector3d(-5, 0.0, 0.5)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)
rover.Initialize(chrono.ChFramed(init_pos, init_rot))






manager = sens.ChSensorManager(system)



def get_rover_chassis(curiosity_rover):
    for method_name in ("GetChassisBody", "GetChassis", "GetRoverBody"):
        if hasattr(curiosity_rover, method_name):
            body = getattr(curiosity_rover, method_name)()
            if body is not None:
                return body
    raise RuntimeError("Could not find a valid chassis body accessor for the Curiosity rover.")

chassis_body = get_rover_chassis(rover)


lidar_update_rate = 10.0          
horizontal_samples = 720
vertical_samples = 16
horizontal_fov = 2.0 * math.pi   
max_vert_angle = math.radians(10)
min_vert_angle = math.radians(-30)
max_lidar_range = 30.0



lidar_offset_pose = chrono.ChFramed(
    chrono.ChVector3d(0.8, 0.0, 0.7),
    chrono.ChQuaterniond(1, 0, 0, 0)
)

lidar = sens.ChLidarSensor(
    chassis_body,
    lidar_update_rate,
    lidar_offset_pose,
    horizontal_samples,
    vertical_samples,
    horizontal_fov,
    max_vert_angle,
    min_vert_angle,
    max_lidar_range
)

lidar.SetName("Curiosity_Chassis_Lidar")
lidar.SetLag(0.0)
lidar.SetCollectionWindow(1.0 / lidar_update_rate)


lidar.PushFilter(sens.ChFilterDIAccess())       
lidar.PushFilter(sens.ChFilterPCfromDepth())    
lidar.PushFilter(sens.ChFilterXYZIAccess())     

manager.AddSensor(lidar)






vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Curiosity rover with lidar sensor")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(-4, 4, 3), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()
vis.AddLightWithShadow(
    chrono.ChVector3d(1.5, -2.5, 5.5),
    chrono.ChVector3d(0, 0, 0),
    3,
    4,
    10,
    40,
    512
)









time_step = 1e-3

while vis.Run():
    
    driver.SetSteering(0.0)

    
    if hasattr(driver, "SetThrottle"):
        driver.SetThrottle(0.4)

    
    rover.Update()
    manager.Update()

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    system.DoStepDynamics(time_step)