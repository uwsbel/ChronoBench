import os
import math
import pychrono as chrono
import pychrono.robot as robot
import pychrono.irrlicht as chronoirr

# System setup — Curiosity uses ChSystemNSC + Bullet collision
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # required for contact
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))  # Z-down gravity
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)          # rover contact envelope
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)            # rover contact margin

# Ground — flat rigid box, top surface at z=0 (Curiosity ground convention)
ground_mat = chrono.ChContactMaterialNSC()
ground = chrono.ChBodyEasyBox(40, 20, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, -0.5))                         # top at z=0
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)

# Long box obstacle for the rover to cross — placed ahead of the rover's path
obs_mat = chrono.ChContactMaterialNSC()
obstacle = chrono.ChBodyEasyBox(2.0, 3.0, 0.15, 2000, True, True, obs_mat)
obstacle.SetPos(chrono.ChVector3d(0, 0, 0.075))                      # sits on ground (z=0 top)
obstacle.SetFixed(True)
obstacle.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/bluewhite.png"))
system.Add(obstacle)

# Curiosity rover — init pos changed to (-5, 0.0, 0)
rover = robot.Curiosity(system)
driver = robot.CuriosityDCMotorControl()
rover.SetDriver(driver)
init_pos = chrono.ChVector3d(-5, 0.0, 0)                             # updated spawn position
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                          # identity orientation
rover.Initialize(chrono.ChFramed(init_pos, init_rot))

# Irrlicht visualization — Initialize() FIRST, scene elements AFTER
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Curiosity rover - box obstacle")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(-5, 6, 2), chrono.ChVector3d(-5, 0, 0))  # camera looking at rover
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5),
                       chrono.ChVector3d(0, 0, 0.5),
                       3, 4, 10, 40, 512)

# Simulation parameters
time_step = 1e-3                                                      # 1 ms time step
sim_end = 20.0                                                        # run for 20 s
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))         # untagged cadence constant


# Main loop — zero steering (drive straight forward)
while vis.Run() and system.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        driver.SetSteering(0.0)                                       # zero steering — move straight
        rover.Update()                                                # propagate steering command
        system.DoStepDynamics(time_step)                             # advance physics
        if system.GetChTime() >= sim_end:
            break
