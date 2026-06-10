import os
import math
import pychrono.core as chrono
import pychrono.robot as robot
import pychrono.irrlicht as chronoirr

time_step = 1e-3                                                       # integration step for the rover
sim_end = 20.0                                                        # total simulated time
max_steering = math.pi / 6                                           # practical steering limit (rad)

system = chrono.ChSystemNSC()                                        # rovers use a non-smooth system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # Bullet collision (required for contact)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))  # Z-down gravity, g = 9.81
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)          # small contact envelope for rover wheels
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)            # small contact margin for rover wheels

ground_mat = chrono.ChContactMaterialNSC()                           # NSC contact material for the ground
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)  # 20x20x1 m collidable box
ground.SetPos(chrono.ChVector3d(0, 0, -0.5))                         # top surface at z = 0 under the rover
ground.SetFixed(True)                                                # ground is static terrain
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))  # rigid terrain texture
system.Add(ground)                                                   # add the ground to the system

rover = robot.Curiosity(system)                                      # built-in Curiosity Mars rover (owns its bodies)
driver = robot.CuriosityDCMotorControl()                             # DC-motor steering control driver
rover.SetDriver(driver)                                              # attach the driver BEFORE Initialize
init_pos = chrono.ChVector3d(0, 0.2, 0)                              # rover spawn position
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                          # identity orientation (w,x,y,z)
rover.Initialize(chrono.ChFramed(init_pos, init_rot))               # place the rover on the terrain

vis = chronoirr.ChVisualSystemIrrlicht()                             # Irrlicht render window
vis.AttachSystem(system)                                             # bind the system's visual assets
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)                   # Z-up camera convention
vis.SetWindowSize(1280, 720)                                        # window resolution
vis.SetWindowTitle("Curiosity rover - Rigid terrain")              # window title
vis.Initialize()                                                    # create the Irrlicht device FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))   # PyChrono logo overlay
vis.AddSkyBox()                                                     # sky box background
vis.AddCamera(chrono.ChVector3d(0, 3, 3), chrono.ChVector3d(0, 0, 0))  # eye and look-at target
vis.AddTypicalLights()                                             # standard scene lights
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5), chrono.ChVector3d(0, 0, 0),
                       3, 4, 10, 40, 512)                          # shadow-casting light: pos, aim, radius, near, far, angle, res

render_fps = 50.0                                                  # review-video frame rate
render_every = max(1, round(1.0 / (render_fps * time_step)))      # physics steps between captured frames
step = 0                                                          # physics step counter (render cadence)

while vis.Run() and system.GetChTime() < sim_end:
    time = system.GetChTime()                                    # current simulated time

    steering = 0.0                                               # default: drive straight
    if 2 < time < 7:                                             # ramp the steering in
        steering = max_steering * (time - 2) / 5
    elif 7 < time < 12:                                          # ramp the steering back out
        steering = max_steering * (12 - time) / 5
    driver.SetSteering(steering)                                 # command the steering angle
    rover.Update()                                              # propagate command into the motors

    vis.BeginScene()                                            # start the render frame
    vis.Render()                                                # draw the scene
    vis.EndScene()                                              # finish the render frame
    system.DoStepDynamics(time_step)                           # advance the physics one step
    step += 1                                                  # advance the step counter
