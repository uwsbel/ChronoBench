import os
import math
import pychrono as chrono
import pychrono.robot as robot
import pychrono.irrlicht as chronoirr

# ---- system (rovers use NSC + Bullet collision) ----
system = chrono.ChSystemNSC()                                          # non-smooth system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)   # Bullet collision (contact scene)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))   # g = 9.81 down (Z-up)
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)           # small contact envelope
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)             # small contact margin

# ---- ground (fixed box, top under the spawn) ----
ground_mat = chrono.ChContactMaterialNSC()                            # NSC contact material for ground
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)  # 20x20x1 textured box
ground.SetPos(chrono.ChVector3d(0, 0, -0.6))                          # ground position (raised from -1)
ground.SetFixed(True)                                                 # ground does not move
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))  # concrete texture
system.Add(ground)                                                    # add ground to system

# ---- TurtleBot (pose in constructor, no driver, no Update) ----
init_pos = chrono.ChVector3d(0, 0.2, 0)                               # spawn just above the ground top
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                           # identity orientation (w,x,y,z)
robot_tb = robot.TurtleBot(system, init_pos, init_rot)               # rover owns its own bodies
robot_tb.Initialize()                                                 # no-arg init builds wheels/motors

LEFT_DRIVE_WHEEL = 0                                                  # WheelID 0 = LEFT
RIGHT_DRIVE_WHEEL = 1                                                 # WheelID 1 = RIGHT


def move(mode):                                                       # set per-wheel motor speeds by mode
    if mode == "straight":                                           # both wheels forward
        robot_tb.SetMotorSpeed(-math.pi, LEFT_DRIVE_WHEEL)
        robot_tb.SetMotorSpeed(-math.pi, RIGHT_DRIVE_WHEEL)
    elif mode == "left":                                            # pivot left: left wheel stops
        robot_tb.SetMotorSpeed(0, LEFT_DRIVE_WHEEL)
        robot_tb.SetMotorSpeed(-math.pi, RIGHT_DRIVE_WHEEL)
    elif mode == "right":                                           # pivot right: right wheel stops
        robot_tb.SetMotorSpeed(-math.pi, LEFT_DRIVE_WHEEL)
        robot_tb.SetMotorSpeed(0, RIGHT_DRIVE_WHEEL)
    else:                                                          # reject any unknown mode
        raise ValueError("Invalid mode: " + str(mode))


# ---- Irrlicht visualization (Initialize first, then scene elements; NO grid) ----
vis = chronoirr.ChVisualSystemIrrlicht()                             # Irrlicht render window
vis.AttachSystem(system)                                             # bind the system's visual assets
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)                   # Z-up camera convention
vis.SetWindowSize(1280, 720)                                        # window resolution
vis.SetWindowTitle("TurtleBot - Rigid terrain")                    # window title
vis.Initialize()                                                    # create device BEFORE scene nodes
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))   # PyChrono logo
vis.AddSkyBox()                                                     # sky box
vis.AddCamera(chrono.ChVector3d(0, 2.5, 1.5), chrono.ChVector3d(0, 0, 1))  # eye, target
vis.AddTypicalLights()                                              # standard two-light setup
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5), chrono.ChVector3d(0, 0, 0.5),
                       3, 4, 10, 40, 512)                            # shadow light: pos, aim, r, near, far, angle, res

# ---- time stepping ----
time_step = 2e-3                                                     # TurtleBot uses 2e-3
sim_end = 15.0                                                       # 5 s straight + 5 s left + 5 s right
render_fps = 50.0                                                    # review frame rate
render_every = max(1, round(1.0 / (render_fps * time_step)))        # untagged cadence constant
action = None                                                       # current action label (for print on change)

while vis.Run() and system.GetChTime() < sim_end:
    vis.BeginScene()                                               # begin frame
    vis.Render()                                                   # draw scene
    vis.EndScene()                                                 # end frame
    for _ in range(render_every):
        t = system.GetChTime()                                     # current sim time
        if t < 5.0:                                                # first 5 s: drive straight
            if action != "straight":
                action = "straight"
                print("Moving straight")
            move("straight")
        elif t < 10.0:                                              # next 5 s: turn left
            if action != "left":
                action = "left"
                print("Turning left")
            move("left")
        else:                                                      # thereafter: turn right
            if action != "right":
                action = "right"
                print("Turning right")
            move("right")
        system.DoStepDynamics(time_step)                           # advance one physics step
        if system.GetChTime() >= sim_end:
            break
