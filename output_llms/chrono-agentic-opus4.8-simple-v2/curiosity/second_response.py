import os
import pychrono as chrono
import pychrono.robot as robot
import pychrono.irrlicht as chronoirr

# Curiosity Mars rover on rigid terrain, crossing a long box obstacle.
system = chrono.ChSystemNSC()                                          # NSC system (rovers use NSC)
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)   # Bullet collision (contact scene)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))   # g = 9.81 m/s^2 down (Z-up)
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)           # small contact envelope
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)             # small contact margin

# Rigid ground: a fixed box whose top surface sits at z = 0.
ground_mat = chrono.ChContactMaterialNSC()                            # NSC contact material for ground
ground = chrono.ChBodyEasyBox(30, 20, 1, 1000, True, True, ground_mat) # 30x20x1 m box, with visual + collision
ground.SetPos(chrono.ChVector3d(0, 0, -0.5))                          # drop so top is at z = 0 (Curiosity)
ground.SetFixed(True)                                                 # ground is static
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))  # concrete texture
system.Add(ground)                                                    # add ground to the system

# Long box obstacle for the rover to cross, placed ahead of the spawn.
obstacle_mat = chrono.ChContactMaterialNSC()                          # NSC contact material for obstacle
obstacle = chrono.ChBodyEasyBox(0.25, 6.0, 0.12, 1000, True, True, obstacle_mat)  # long, low ridge (X,Y,Z)
obstacle.SetPos(chrono.ChVector3d(0, 0, 0.06))                        # centered, resting on the ground (top at 0.12)
obstacle.SetFixed(True)                                               # fixed so the rover climbs over it
obstacle.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))  # visible color
system.Add(obstacle)                                                  # add obstacle to the system

# Curiosity rover: it owns its bodies; attach a DC-motor driver, then Initialize.
rover = robot.Curiosity(system)                                       # build the Curiosity rover on this system
driver = robot.CuriosityDCMotorControl()                             # DC-motor steering driver
rover.SetDriver(driver)                                               # attach driver BEFORE Initialize

init_pos = chrono.ChVector3d(-5, 0.0, 0)                             # spawn behind the obstacle
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                          # identity orientation (w,x,y,z)
rover.Initialize(chrono.ChFramed(init_pos, init_rot))               # Initialize takes a ChFramed (pos + rot)

# Irrlicht visualization window (Initialize first, then scene elements; NO grid).
vis = chronoirr.ChVisualSystemIrrlicht()                             # create the Irrlicht visual system
vis.AttachSystem(system)                                             # bind the physical system
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)                   # Z is up
vis.SetWindowSize(1280, 720)                                        # window pixel size
vis.SetWindowTitle("Curiosity rover - crossing a box obstacle")     # window title
vis.Initialize()                                                    # create the device FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))    # PyChrono logo
vis.AddSkyBox()                                                     # sky box
vis.AddCamera(chrono.ChVector3d(0, 3.0, 1.5), chrono.ChVector3d(0, 0, 1))  # eye, look-at
vis.AddTypicalLights()                                             # standard lights
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5), chrono.ChVector3d(0, 0, 0.5),
                       3, 4, 10, 40, 512)                          # shadow-casting light

# Time-stepping configuration.
time_step = 1e-3                                                   # 1 ms physics step (Curiosity)
sim_end = 16.0                                                     # total simulated time
render_fps = 50.0                                                  # render/capture cadence
render_every = max(1, round(1.0 / (render_fps * time_step)))      # physics steps per rendered frame

# Main loop: drive forward with zero steering and cross the box obstacle.
while vis.Run() and system.GetChTime() < sim_end:
    vis.BeginScene()                                              # begin frame
    vis.Render()                                                  # render the scene
    vis.EndScene()                                                # end frame
    for _ in range(render_every):
        driver.SetSteering(0.0)                                   # zero steering -> straight ahead
        rover.Update()                                           # propagate command into the motors
        system.DoStepDynamics(time_step)                        # advance one physics step
        if system.GetChTime() >= sim_end:
            break
