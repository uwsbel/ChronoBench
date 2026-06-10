import os
import math
import pychrono.core as chrono
import pychrono.robot as robot
import pychrono.irrlicht as chronoirr

system = chrono.ChSystemNSC()                                           # rovers use NSC
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)     # required for contact
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))     # Z-down gravity
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)            # small contact envelope
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)             # small contact margin

ground_mat = chrono.ChContactMaterialNSC()                            # NSC ground material
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)  # 20x20x1 rigid ground
ground.SetPos(chrono.ChVector3d(0, 0, -0.5))                          # box top at z=0 (Curiosity)
ground.SetFixed(True)                                                  # ground is fixed
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))  # terrain texture
system.Add(ground)                                                     # add ground to system

rover = robot.Curiosity(system)                                       # built-in Curiosity rover
driver = robot.CuriosityDCMotorControl()                             # DC-motor steering driver
rover.SetDriver(driver)                                              # attach driver before Initialize

init_pos = chrono.ChVector3d(0, 0, 0.5)                               # spawn above ground
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                          # identity rotation (w,x,y,z)
rover.Initialize(chrono.ChFramed(init_pos, init_rot))               # place the rover

vis = chronoirr.ChVisualSystemIrrlicht()                             # Irrlicht window
vis.AttachSystem(system)                                             # bind the system
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)                   # Z-up camera
vis.SetWindowSize(1280, 720)                                        # window resolution
vis.SetWindowTitle("Curiosity rover - Rigid terrain")              # window title
vis.Initialize()                                                    # create device FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))   # corner logo
vis.AddSkyBox()                                                     # sky box
vis.AddCamera(chrono.ChVector3d(0, 3.0, 1.5), chrono.ChVector3d(0, 0, 1))  # eye, target
vis.AddTypicalLights()                                              # standard lights
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5), chrono.ChVector3d(0, 0, 0.5),
                       3, 4, 10, 40, 512)                           # shadow-casting light

time_step = 1e-3                                                    # integration step
sim_end = 15.0                                                      # simulation duration
max_steering = math.pi / 6                                         # practical max steering
render_fps = 50.0                                                  # review frame rate
render_every = max(1, round(1.0 / (render_fps * time_step)))      # cadence constant
while vis.Run() and system.GetChTime() < sim_end:
    vis.BeginScene()                                              # begin render
    vis.Render()                                                  # draw scene
    vis.EndScene()                                                # end render
    for _ in range(render_every):
        time = system.GetChTime()                                # current sim time
        steering = 0.0                                            # default: drive straight
        if 2 < time < 7:
            steering = max_steering * (time - 2) / 5             # ramp steering in
        elif 7 < time < 12:
            steering = max_steering * (12 - time) / 5           # ramp steering out
        driver.SetSteering(steering)                             # command steering
        rover.Update()                                           # propagate command to motors
        system.DoStepDynamics(time_step)                        # advance one step
        if system.GetChTime() >= sim_end:
            break
