import math
import pychrono.core as chrono
import pychrono.robot as robot
import pychrono.irrlicht as chronoirr

# Chrono system: rovers use a non-smooth (NSC) system with Bullet collision.
system = chrono.ChSystemNSC()                                          # NSC system for the rover
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)   # contact needs Bullet
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))   # g = 9.81 m/s^2, Z-down
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)           # small contact envelope
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)             # small contact margin

# Rigid terrain: a fixed box whose top surface sits at z = -0.5 under the spawn.
ground_mat = chrono.ChContactMaterialNSC()                            # NSC contact material
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)  # 20x20x1 m ground box
ground.SetPos(chrono.ChVector3d(0, 0, -1))                            # top surface at z = -0.5
ground.SetFixed(True)                                                 # terrain does not move
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))  # concrete look
system.Add(ground)                                                    # add the ground to the system

# Viper rover: the class owns its bodies; attach the DC-motor driver before Initialize.
rover = robot.Viper(system)                                           # 6-wheel suspension rover
driver = robot.ViperDCMotorControl()                                  # DC-motor steering driver
rover.SetDriver(driver)                                               # attach driver BEFORE Initialize

init_pos = chrono.ChVector3d(0, 0.2, 0)                               # spawn just above the terrain
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                           # identity orientation (w,x,y,z)
rover.Initialize(chrono.ChFramed(init_pos, init_rot))                 # Initialize takes a ChFramed

# Irrlicht visualization (Initialize first, then add scene elements; NO grid).
vis = chronoirr.ChVisualSystemIrrlicht()                              # Irrlicht window
vis.AttachSystem(system)                                              # bind the system's visuals
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)                     # Z-up world
vis.SetWindowSize(1280, 720)                                          # window resolution
vis.SetWindowTitle("Viper rover - Rigid terrain")                    # window title
vis.Initialize()                                                     # create the device first
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))     # PyChrono logo
vis.AddSkyBox()                                                      # sky box backdrop
vis.AddCamera(chrono.ChVector3d(0, 2.5, 1.5), chrono.ChVector3d(0, 0, 1))  # eye, look-at
vis.AddTypicalLights()                                               # standard lighting
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5), chrono.ChVector3d(0, 0, 0.5),
                       3, 4, 10, 40, 512)                            # pos, aim, radius, near, far, angle, res

# Simulation parameters.
time_step = 1e-3                                                     # 1 ms physics step
sim_end = 14.0                                                       # total simulated time (s)
max_steering = math.pi / 6                                           # practical max steering (~30 deg)
render_fps = 50.0                                                    # review render cadence
render_every = max(1, round(1.0 / (render_fps * time_step)))        # physics steps per rendered frame

# Main loop: steering ramps in, holds, and ramps back out over the run.
while vis.Run() and system.GetChTime() < sim_end:
    vis.BeginScene()                                                # begin frame
    vis.Render()                                                    # draw the scene
    vis.EndScene()                                                  # finish frame
    for _ in range(render_every):
        t = system.GetChTime()                                      # current sim time
        steering = 0.0                                              # default: drive straight
        if 2 < t < 7:
            steering = max_steering * (t - 2) / 5                   # gradually steer in
        elif 7 < t < 12:
            steering = max_steering * (12 - t) / 5                  # gradually steer back out
        driver.SetSteering(steering)                               # command the steering angle
        rover.Update()                                             # propagate command into motors
        system.DoStepDynamics(time_step)                           # advance one physics step
        if system.GetChTime() >= sim_end:
            break
