import os
import math
import pychrono as chrono
import pychrono.robot as robot
import pychrono.irrlicht as chronoirr

time_step = 2e-3                                                       # TurtleBot uses 2e-3
sim_end = 15.0                                                         # 5 s straight + 5 s left + rest right

system = chrono.ChSystemNSC()                                         # rovers use NSC
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)   # contact scene needs Bullet
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))   # g = 9.81 down (Z-up)
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)           # small rover contact envelope
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)            # small rover contact margin

ground_mat = chrono.ChContactMaterialNSC()                            # NSC contact material for ground
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)  # 20x20 fixed floor, 1 m thick
ground.SetPos(chrono.ChVector3d(0, 0, -0.6))                          # ground top at z = -0.1
ground.SetFixed(True)                                                 # static floor
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))  # concrete look
system.Add(ground)                                                    # add floor to system

init_pos = chrono.ChVector3d(0, 0.2, 0)                               # spawn slightly above ground
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                           # identity orientation (w,x,y,z)
robot_tb = robot.TurtleBot(system, init_pos, init_rot)               # pose passed in constructor
robot_tb.Initialize()                                                 # no-arg init builds bodies/motors

LEFT_DRIVE_WHEEL = 0                                                   # WheelID 0 = LEFT
RIGHT_DRIVE_WHEEL = 1                                                  # WheelID 1 = RIGHT
wheel_speed = math.pi                                                  # nominal wheel rad/s

def move(mode):                                                        # set wheel speeds per movement mode
    if mode == "straight":                                            # both wheels forward
        robot_tb.SetMotorSpeed(-wheel_speed, LEFT_DRIVE_WHEEL)
        robot_tb.SetMotorSpeed(-wheel_speed, RIGHT_DRIVE_WHEEL)
    elif mode == "left":                                             # pivot left: left wheel stops
        robot_tb.SetMotorSpeed(0, LEFT_DRIVE_WHEEL)
        robot_tb.SetMotorSpeed(-wheel_speed, RIGHT_DRIVE_WHEEL)
    elif mode == "right":                                            # pivot right: right wheel stops
        robot_tb.SetMotorSpeed(-wheel_speed, LEFT_DRIVE_WHEEL)
        robot_tb.SetMotorSpeed(0, RIGHT_DRIVE_WHEEL)
    else:
        raise ValueError("invalid mode: " + str(mode))               # guard against bad mode

vis = chronoirr.ChVisualSystemIrrlicht()                              # Irrlicht render window
vis.AttachSystem(system)                                              # bind the system's visuals
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)                    # Z-up camera (before Initialize)
vis.SetWindowSize(1280, 720)                                         # window resolution
vis.SetWindowTitle("TurtleBot - Rigid terrain")                     # window title
vis.Initialize()                                                     # create device FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))    # logo (after Initialize)
vis.AddSkyBox()                                                      # sky box
vis.AddCamera(chrono.ChVector3d(0.3, 1.6, 1.1), chrono.ChVector3d(0.3, 0, 0))  # eye, target (close, low)
vis.AddTypicalLights()                                               # standard lights
vis.AddLight(chrono.ChVector3d(1.5, -2.5, 5.5), 12, chrono.ChColor(0.9, 0.9, 0.9))  # extra point light

render_fps = 50.0                                                    # review render cadence
render_every = max(1, round(1.0 / (render_fps * time_step)))         # untagged cadence constant

current = None                                                       # track last commanded mode
time = 0.0                                                           # elapsed sim time
while vis.Run() and system.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        time = system.GetChTime()                                   # current sim time
        if time < 5.0:                                              # first 5 s: drive straight
            mode = "straight"
        elif time < 10.0:                                          # next 5 s: turn left
            mode = "left"
        else:                                                      # thereafter: turn right
            mode = "right"
        if mode != current:                                        # only re-command on change
            move(mode)                                             # apply the new wheel speeds
            print("Robot action:", mode)                           # report current action
            current = mode
        system.DoStepDynamics(time_step)                           # advance one physics step
        if system.GetChTime() >= sim_end:
            break
