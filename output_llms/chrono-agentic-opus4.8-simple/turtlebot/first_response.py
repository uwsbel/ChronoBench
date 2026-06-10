import math
import pychrono as chrono
import pychrono.robot as robot
import pychrono.irrlicht as chronoirr

system = chrono.ChSystemNSC()                                        # rovers use NSC
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # contact -> Bullet
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))  # Z-down gravity
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)         # small contact envelope
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)          # small contact margin

ground_mat = chrono.ChContactMaterialNSC()                          # NSC ground material
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)  # rigid terrain box
ground.SetPos(chrono.ChVector3d(0, 0, -1))                          # top surface at z=-0.5
ground.SetFixed(True)                                               # static terrain
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))  # terrain texture
system.Add(ground)                                                  # add ground to system

init_pos = chrono.ChVector3d(0, 0, -0.45)                          # wheels rest on terrain top
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                         # identity orientation
robot_tb = robot.TurtleBot(system, init_pos, init_rot)             # pose passed in constructor
robot_tb.Initialize()                                              # no-arg Initialize

vis = chronoirr.ChVisualSystemIrrlicht()                           # Irrlicht renderer
vis.AttachSystem(system)                                           # bind the system
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)                 # Z-up camera
vis.SetWindowSize(1280, 720)                                       # window resolution
vis.SetWindowTitle("Turtlebot Robot - Rigid terrain")            # window title
vis.Initialize()                                                  # device first, then scene
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))  # logo overlay
vis.AddSkyBox()                                                    # sky box
vis.AddCamera(chrono.ChVector3d(0, 1.5, 0.2), chrono.ChVector3d(0, 0, 0.2))  # eye, target
vis.AddTypicalLights()                                             # standard lights
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5), chrono.ChVector3d(0, 0, 0.5),
                       3, 4, 10, 40, 512)                          # pos, aim, radius, near, far, angle, res

time_step = 2e-3                                                   # TurtleBot uses 2e-3
sim_end = 10.0                                                     # total sim duration
render_fps = 50.0                                                  # review frame rate
render_every = max(1, round(1.0 / (render_fps * time_step)))      # cadence constant (untagged)


time = 0                                                          # sim clock for motor schedule
while vis.Run() and system.GetChTime() < sim_end:
    # at time = 1 s, start left turn; Left drive wheel = 0, Right drive wheel = 1
    if abs(time - 1.0) < 1e-4:
        robot_tb.SetMotorSpeed(-0, 0)                              # left wheel stop
        robot_tb.SetMotorSpeed(-math.pi, 1)                       # right wheel forward -> left turn
    # at time = 2 s, start right turn
    if abs(time - 2.0) < 1e-4:
        robot_tb.SetMotorSpeed(-math.pi, 0)                      # left wheel forward -> right turn
        robot_tb.SetMotorSpeed(-0, 1)                             # right wheel stop
    time = time + time_step                                       # advance schedule clock
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(time_step)                              # advance physics one step
