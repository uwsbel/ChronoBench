import os
import math
import pychrono as chrono
import pychrono.robot as robot
import pychrono.irrlicht as chronoirr

system = chrono.ChSystemNSC()  # NSC for rover contact
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # required for wheel/terrain contact
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))  # gravity Z-down
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)  # collision envelope
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)    # collision margin

ground_mat = chrono.ChContactMaterialNSC()  # NSC contact material for ground
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)  # flat ground
ground.SetPos(chrono.ChVector3d(0, 0, -0.6))  # ground center at -0.6 (top surface at -0.1)
ground.SetFixed(True)  # ground is static
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))  # ground texture
system.Add(ground)

init_pos = chrono.ChVector3d(0, 0.2, 0)           # TurtleBot initial position
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)        # identity rotation (w,x,y,z)
robot_tb = robot.TurtleBot(system, init_pos, init_rot)  # pose passed to constructor
robot_tb.Initialize()                                    # no-arg Initialize

LEFT_DRIVE_WHEEL  = robot.LD   # left drive wheel ID = 0
RIGHT_DRIVE_WHEEL = robot.RD   # right drive wheel ID = 1

def move(mode):
    if mode == "straight":
        robot_tb.SetMotorSpeed(-math.pi, LEFT_DRIVE_WHEEL)   # both wheels forward
        robot_tb.SetMotorSpeed(-math.pi, RIGHT_DRIVE_WHEEL)
    elif mode == "left":
        robot_tb.SetMotorSpeed(0, LEFT_DRIVE_WHEEL)           # left wheel stopped -> turn left
        robot_tb.SetMotorSpeed(-math.pi, RIGHT_DRIVE_WHEEL)
    elif mode == "right":
        robot_tb.SetMotorSpeed(-math.pi, LEFT_DRIVE_WHEEL)   # right wheel stopped -> turn right
        robot_tb.SetMotorSpeed(0, RIGHT_DRIVE_WHEEL)
    else:
        raise ValueError("Invalid mode: " + str(mode))

vis = chronoirr.ChVisualSystemIrrlicht()  # Irrlicht visualization system
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)  # Z-up
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("TurtleBot - Differential Drive")
vis.Initialize()  # Initialize FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 2.5, 1.5), chrono.ChVector3d(0, 0, 1))  # camera looking at robot
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5), chrono.ChVector3d(0, 0, 0.5),
                       3, 4, 10, 40, 512)  # shadow light

time_step = 2e-3   # TurtleBot uses 2e-3 step size
sim_end = 20.0     # run for 20 s total (5 straight + 5 left + 10 right)
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))  # cadence constant


move("straight")  # start moving straight
print("Moving straight")  # initial action

while vis.Run() and system.GetChTime() < sim_end:
    vis.BeginScene(); vis.Render(); vis.EndScene()
    for _ in range(render_every):
        t = system.GetChTime()
        if t < 5.0:  # first 5 seconds: straight
            move("straight")
        elif t < 10.0:  # next 5 seconds: turn left
            move("left")
        else:  # thereafter: turn right
            move("right")
        system.DoStepDynamics(time_step)
        # print mode transitions
        new_t = system.GetChTime()
        if abs(new_t - 5.0) < time_step:
            print("Turning left")
        if abs(new_t - 10.0) < time_step:
            print("Turning right")
        if new_t >= sim_end:
            break
