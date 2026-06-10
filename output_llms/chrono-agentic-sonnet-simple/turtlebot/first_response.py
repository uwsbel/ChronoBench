import os
import math
import pychrono as chrono
import pychrono.robot as robot
import pychrono.irrlicht as chronoirr

# System setup — NSC with Bullet collision for rover/terrain contact
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)   # required: robot-terrain contact
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))  # g = 9.81 downward
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)           # small contact envelope
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)             # small contact margin

# Ground — rigid box, top surface at z = 0; robot spawns above it
ground_mat = chrono.ChContactMaterialNSC()
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)  # 20x20x1 m slab
ground.SetPos(chrono.ChVector3d(0, 0, -0.5))                          # top surface at z = 0
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)

# TurtleBot robot — pose passed to constructor; Initialize() takes no argument
init_pos = chrono.ChVector3d(0, 0, 0.2)                              # spawn slightly above ground
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                          # identity (w,x,y,z)
robot_tb = robot.TurtleBot(system, init_pos, init_rot)               # system owns all robot bodies
robot_tb.Initialize()                                                 # no-arg for TurtleBot

LEFT_WHEEL  = 0   # wheel id: 0 = LEFT
RIGHT_WHEEL = 1   # wheel id: 1 = RIGHT

# Irrlicht visualization — Initialize() FIRST, scene elements AFTER
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)                     # Z-up world
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("TurtleBot - Rigid Terrain")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, -3, 2), chrono.ChVector3d(0, 0, 0))  # rear-above view
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5),
                       chrono.ChVector3d(0, 0, 0.5), 3, 4, 10, 40, 512)

# Simulation parameters
time_step = 2e-3           # TurtleBot uses 2 ms
sim_end   = 10.0           # run for 10 s
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))          # cadence constant (untagged)

# Review-only recording setup

# Simulation loop — TurtleBot motor control: left and right turns at specified times
while vis.Run() and system.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        t = system.GetChTime()
        # Motor speed commands: pivot left at t=2 s, pivot right at t=5 s
        if abs(t - 2.0) < time_step:
            robot_tb.SetMotorSpeed(0,           LEFT_WHEEL)           # stop left wheel
            robot_tb.SetMotorSpeed(-math.pi,    RIGHT_WHEEL)          # right drives → left turn
        if abs(t - 5.0) < time_step:
            robot_tb.SetMotorSpeed(-math.pi,    LEFT_WHEEL)           # left drives → right turn
            robot_tb.SetMotorSpeed(0,           RIGHT_WHEEL)          # stop right wheel
        system.DoStepDynamics(time_step)                              # advance physics (no Update for TurtleBot)
        if system.GetChTime() >= sim_end:
            break
