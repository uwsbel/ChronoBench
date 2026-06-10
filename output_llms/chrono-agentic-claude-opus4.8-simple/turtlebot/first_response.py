import os
import math
import pychrono as chrono
import pychrono.robot as turtlebot
import pychrono.irrlicht as chronoirr

system = chrono.ChSystemNSC()                                         # NSC system for the rover
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)   # Bullet collision (rover<->ground contact)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))   # Z-up world, g downward
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)           # collision envelope
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)             # collision margin

ground_mat = chrono.ChContactMaterialNSC()                            # NSC contact material for terrain
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)  # rigid terrain box
ground.SetPos(chrono.ChVector3d(0, 0, -1))                            # top surface at z=-0.5
ground.SetFixed(True)                                                 # static terrain
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))  # concrete texture
system.Add(ground)

init_pos = chrono.ChVector3d(0, 0.2, 0)                               # spawn just above terrain
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                           # identity orientation
robot = turtlebot.TurtleBot(system, init_pos, init_rot)               # TurtleBot owns its bodies; pose in ctor
robot.Initialize()                                                    # no-arg Initialize for TurtleBot

vis = chronoirr.ChVisualSystemIrrlicht()                              # Irrlicht real-time window
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)                     # Z-up camera
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("TurtleBot robot - Rigid terrain")
vis.Initialize()                                                      # Initialize FIRST, then scene elements
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 1.5, 0.2), chrono.ChVector3d(0, 0, 0.2))  # follow camera
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5), chrono.ChVector3d(0, 0, 0.5),
                       3, 4, 10, 40, 512)                             # pos, aim, radius, near, far, angle, res
# vis.EnableShadows()   # truth keeps this COMMENTED; no-arg form raises TypeError in 9.0.0

time_step = 2e-3                                                      # TurtleBot uses 2e-3
render_fps = 50.0                                                     # review video frame rate
render_every = max(1, round(1.0 / (render_fps * time_step)))         # render cadence constant
LEFT_DRIVE_WHEEL = 0                                                  # WheelID: 0 = LEFT
RIGHT_DRIVE_WHEEL = 1                                                 #          1 = RIGHT


time = 0                                                              # simulation clock
while vis.Run():                                                      # plain real-time loop
    vis.BeginScene(); vis.Render(); vis.EndScene()                   # draw one frame
    for _ in range(render_every):                                    # advance physics between frames
        if abs(time - 1.0) < 1e-4:                                   # at t=1s: pivot left
            robot.SetMotorSpeed(0, LEFT_DRIVE_WHEEL)
            robot.SetMotorSpeed(-math.pi, RIGHT_DRIVE_WHEEL)
        if abs(time - 2.0) < 1e-4:                                   # at t=2s: pivot right
            robot.SetMotorSpeed(-math.pi, LEFT_DRIVE_WHEEL)
            robot.SetMotorSpeed(0, RIGHT_DRIVE_WHEEL)
        time += time_step                                            # advance clock
        system.DoStepDynamics(time_step)                            # step dynamics (no robot.Update for TurtleBot)
