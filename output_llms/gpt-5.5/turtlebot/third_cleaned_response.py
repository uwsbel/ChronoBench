import os
import math
import numpy as np

import pychrono as chrono
import pychrono.robot as turtlebot
import pychrono.sensor as sens
from pychrono import irrlicht as chronoirr





system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))

chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)





ground_mat = chrono.ChContactMaterialNSC()
ground_mat.SetFriction(0.8)

box_mat = chrono.ChContactMaterialNSC()
box_mat.SetFriction(0.6)






ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, -0.6))
ground.SetFixed(True)

if ground.GetVisualShape(0):
    ground.GetVisualShape(0).SetTexture(
        chrono.GetChronoDataFile("textures/concrete.jpg")
    )

system.Add(ground)






rng = np.random.default_rng(seed=10)

ground_top_z = -0.6 + 0.5  

for i in range(5):
    sx = rng.uniform(0.25, 0.45)
    sy = rng.uniform(0.25, 0.45)
    sz = rng.uniform(0.25, 0.55)

    box = chrono.ChBodyEasyBox(sx, sy, sz, 800, True, True, box_mat)

    
    x = rng.uniform(-3.0, 3.0)
    y = rng.uniform(1.0, 5.0)
    z = ground_top_z + sz / 2.0

    box.SetPos(chrono.ChVector3d(x, y, z))
    box.SetRot(chrono.QuatFromAngleZ(rng.uniform(0, 2 * math.pi)))

    if box.GetVisualShape(0):
        box.GetVisualShape(0).SetColor(
            chrono.ChColor(
                float(rng.uniform(0.2, 1.0)),
                float(rng.uniform(0.2, 1.0)),
                float(rng.uniform(0.2, 1.0)),
            )
        )

    system.Add(box)





init_pos = chrono.ChVector3d(0, 0.2, 0)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)

robot = turtlebot.TurtleBot(system, init_pos, init_rot)
robot.Initialize()





def get_turtlebot_chassis_body(tb):
    
    for method_name in ("GetChassisBody", "GetChassis", "GetBody"):
        if hasattr(tb, method_name):
            body = getattr(tb, method_name)()
            if body is not None:
                return body

    raise RuntimeError(
        "Could not find a Turtlebot chassis body accessor. "
        "Expected one of: GetChassisBody(), GetChassis(), or GetBody()."
    )


chassis_body = get_turtlebot_chassis_body(robot)






manager = sens.ChSensorManager(system)


manager.scene.AddPointLight(
    chrono.ChVector3f(0, 0, 5),
    chrono.ChColor(1.0, 1.0, 1.0),
    100.0,
)

lidar_update_rate = 10.0
lidar_offset_pose = chrono.ChFramed(
    chrono.ChVector3d(0.0, 0.0, 0.28),
    chrono.ChQuaterniond(1, 0, 0, 0),
)

horizontal_samples = 720
vertical_samples = 16
horizontal_fov = 2.0 * math.pi
max_vert_angle = math.radians(10.0)
min_vert_angle = math.radians(-10.0)
max_lidar_range = 10.0

lidar = sens.ChLidarSensor(
    chassis_body,
    lidar_update_rate,
    lidar_offset_pose,
    horizontal_samples,
    vertical_samples,
    horizontal_fov,
    max_vert_angle,
    min_vert_angle,
    max_lidar_range,
)

lidar.SetName("Turtlebot Lidar")
lidar.SetLag(0.0)
lidar.SetCollectionWindow(0.0)


lidar.PushFilter(sens.ChFilterDIAccess())
lidar.PushFilter(sens.ChFilterPCfromDepth())
lidar.PushFilter(
    sens.ChFilterVisualizePointCloud(
        640,
        480,
        1.0,
        "Turtlebot Lidar Point Cloud",
    )
)
lidar.PushFilter(sens.ChFilterXYZIAccess())

manager.AddSensor(lidar)






LEFT_DRIVE_WHEEL = 0
RIGHT_DRIVE_WHEEL = 1

drive_speed = -math.pi


def move(mode):
    
    if mode == "straight":
        robot.SetMotorSpeed(drive_speed, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(drive_speed, RIGHT_DRIVE_WHEEL)

    elif mode == "left":
        robot.SetMotorSpeed(0.0, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(drive_speed, RIGHT_DRIVE_WHEEL)

    elif mode == "right":
        robot.SetMotorSpeed(drive_speed, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(0.0, RIGHT_DRIVE_WHEEL)

    else:
        raise ValueError(
            "Invalid Turtlebot motion mode. "
            "Use one of: 'straight', 'left', or 'right'."
        )





vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Turtlebot Robot - Rigid terrain with Lidar")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(
    chrono.ChVector3d(0, 1.5, 0.8),
    chrono.ChVector3d(0, 0, 0.2),
)
vis.AddTypicalLights()
vis.AddLightWithShadow(
    chrono.ChVector3d(1.5, -2.5, 5.5),
    chrono.ChVector3d(0, 0, 0.5),
    3,
    4,
    10,
    40,
    512,
)









time_step = 2e-3

while vis.Run():
    
    move("straight")

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    system.DoStepDynamics(time_step)

    
    manager.Update()