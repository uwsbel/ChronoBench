import os
import pychrono as chrono
import pychrono.robot as robot
import pychrono.irrlicht as chronoirr

# --- system: Curiosity uses NSC + Bullet collision, Z-up gravity ---
system = chrono.ChSystemNSC()                                          # NSC system for the rover
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)   # required for rover<->ground/obstacle contact
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))   # Z-up gravity
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)           # contact envelope
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)             # contact margin

# --- rigid ground: fixed box, top surface at z=0 ---
ground_mat = chrono.ChContactMaterialNSC()                            # NSC contact material for the ground
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)  # 20x20x1 ground box
ground.SetPos(chrono.ChVector3d(0, 0, -0.5))                          # top surface at z=0
ground.SetFixed(True)                                                 # ground is static
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))  # textured ground
system.Add(ground)                                                    # add ground

# --- long box obstacle for the rover to cross ---
obstacle_mat = chrono.ChContactMaterialNSC()                          # NSC contact material for the obstacle
obstacle = chrono.ChBodyEasyBox(0.25, 5, 0.25, 1000, True, True, obstacle_mat)  # long thin bar across the path
obstacle.SetPos(chrono.ChVector3d(0, 0, 0.125))                       # resting on the ground, ahead of the rover
obstacle.SetFixed(True)                                               # fixed obstacle to drive over
obstacle.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))  # textured obstacle
system.Add(obstacle)                                                  # add obstacle

# --- rover: pass system to ctor; SetDriver BEFORE Initialize ---
rover = robot.Curiosity(system)                                       # built-in Curiosity rover (owns its bodies)
driver = robot.CuriosityDCMotorControl()                              # DC-motor control driver
rover.SetDriver(driver)                                               # attach driver BEFORE Initialize
init_pos = chrono.ChVector3d(-5, 0.0, 0)                              # spawn behind the obstacle
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                           # identity orientation (w,x,y,z)
rover.Initialize(chrono.ChFramed(init_pos, init_rot))                 # build rover at placement frame

# --- Irrlicht visualization (Initialize FIRST, scene elements AFTER) ---
vis = chronoirr.ChVisualSystemIrrlicht()                              # Irrlicht visual system
vis.AttachSystem(system)                                              # bind the physical system
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)                     # Z-up camera convention
vis.SetWindowSize(1280, 720)                                          # window resolution
vis.SetWindowTitle("Curiosity rover - Rigid terrain")                 # window title
vis.Initialize()                                                      # create the window FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))      # PyChrono logo overlay
vis.AddSkyBox()                                                       # sky box backdrop
vis.AddCamera(chrono.ChVector3d(0, 3, 3), chrono.ChVector3d(0, 0, 0))  # camera eye + target
vis.AddTypicalLights()                                                # standard lighting
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5), chrono.ChVector3d(0, 0, 0),
                       3, 4, 10, 40, 512)                             # pos, aim, radius, near, far, angle, res
# vis.EnableShadows()   # truth keeps this COMMENTED; no-arg form (item=0) raises TypeError in 9.0.0

# --- simulation parameters ---
time_step = 1e-3                                                      # integration step
render_fps = 50.0                                                     # target playback fps
render_every = max(1, round(1.0 / (render_fps * time_step)))          # render-cadence constant (untagged)
time = 0.0                                                            # simulation clock

# --- main loop: drive forward with zero steering, render ---
while vis.Run():                                                      # plain real-time loop (no time bound)
    time = system.GetChTime()                                         # current sim time

    driver.SetSteering(0.0)                                           # move forward with zero steering input
    rover.Update()                                                    # propagate command into rover motors

    vis.BeginScene(); vis.Render(); vis.EndScene()                    # draw one frame
    for _ in range(render_every):                                     # advance physics between frames
        system.DoStepDynamics(time_step)                              # step the dynamics
