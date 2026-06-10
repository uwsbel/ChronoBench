import os
import math
import pychrono as chrono
import pychrono.robot as robot
import pychrono.irrlicht as chronoirr

system = chrono.ChSystemNSC()                                        # rovers use NSC
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # Bullet collision for wheel/terrain contact
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))  # g = 9.81, Z-down
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)         # small contact envelope
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)          # small contact margin

ground_mat = chrono.ChContactMaterialNSC()                          # NSC contact material for ground
ground = chrono.ChBodyEasyBox(30, 20, 1, 1000, True, True, ground_mat)  # large rigid ground box
ground.SetPos(chrono.ChVector3d(0, 0, -0.5))                        # Curiosity ground: box top at z=0
ground.SetFixed(True)                                               # ground is static
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))  # texture
system.Add(ground)                                                  # add ground to system

obstacle_mat = chrono.ChContactMaterialNSC()                        # NSC material for the obstacle
obstacle = chrono.ChBodyEasyBox(0.4, 8, 0.08, 1000, True, True, obstacle_mat)  # long low box obstacle to cross
obstacle.SetPos(chrono.ChVector3d(0, 0, 0.04))                      # centered ahead, resting on ground top
obstacle.SetFixed(True)                                             # fixed obstacle bar
obstacle.GetVisualShape(0).SetColor(chrono.ChColor(0.6, 0.3, 0.1))  # brown obstacle
system.Add(obstacle)                                                # add obstacle to system

rover = robot.Curiosity(system)                                     # built-in Mars rover, owns its bodies
driver = robot.CuriosityDCMotorControl()                           # DC-motor steering driver
rover.SetDriver(driver)                                             # attach driver BEFORE Initialize
init_pos = chrono.ChVector3d(-5, 0.0, 0)                            # new initial position
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                         # identity orientation (w,x,y,z)
rover.Initialize(chrono.ChFramed(init_pos, init_rot))              # Initialize takes a ChFramed

vis = chronoirr.ChVisualSystemIrrlicht()                           # Irrlicht render window
vis.AttachSystem(system)                                           # bind the system
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)                 # Z-up convention
vis.SetWindowSize(1280, 720)                                      # window resolution
vis.SetWindowTitle("Curiosity rover - crossing obstacle")        # window title
vis.Initialize()                                                  # create device FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png")) # logo
vis.AddSkyBox()                                                   # sky box
vis.AddCamera(chrono.ChVector3d(-7, 3.5, 2), chrono.ChVector3d(-3, 0, 0))  # follow the rover start
vis.AddTypicalLights()                                            # standard lights
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5), chrono.ChVector3d(0, 0, 0.5),
                       3, 4, 10, 40, 512)                          # shadow light (7-positional)

time_step = 1e-3                                                   # Curiosity uses 1e-3
sim_end = 18.0                                                     # enough time to reach and cross obstacle
render_fps = 50.0                                                  # review cadence
render_every = max(1, round(1.0 / (render_fps * time_step)))     # untagged cadence constant
while vis.Run() and system.GetChTime() < sim_end:
    driver.SetSteering(0.0)                                       # drive forward, zero steering
    rover.Update()                                               # REQUIRED after SetSteering
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        system.DoStepDynamics(time_step)                         # advance one step
        if system.GetChTime() >= sim_end:
            break
