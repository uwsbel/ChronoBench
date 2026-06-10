import os
import math
import pychrono as chrono
import pychrono.robot as robot
import pychrono.irrlicht as chronoirr

# --- System: TurtleBot uses non-smooth contact (NSC) with Bullet collision ---
system = chrono.ChSystemNSC()                                          # rover NSC system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)   # contact -> Bullet required
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))   # Z-up, g = 9.81 down
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)           # small contact envelope
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)             # small contact margin

# --- Rigid terrain: fixed ground box, top surface at z = -0.5 under the spawn ---
ground_mat = chrono.ChContactMaterialNSC()                            # NSC contact material for ground
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)  # 20x20x1 m ground box
ground.SetPos(chrono.ChVector3d(0, 0, -0.55))                         # 1 m thick -> top at z = -0.05 under wheels
ground.SetFixed(True)                                                 # ground is static
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))  # concrete look
system.Add(ground)                                                    # attach ground to system

# --- TurtleBot: pose in CONSTRUCTOR, no driver, no Update() ---
init_pos = chrono.ChVector3d(0, 0.2, 0)                               # spawn just above ground top
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                           # identity (w,x,y,z)
robot_tb = robot.TurtleBot(system, init_pos, init_rot)                # rover owns its bodies/motors
robot_tb.Initialize()                                                 # no-arg initialize

LEFT_DRIVE_WHEEL = 0                                                  # WheelID 0 = LEFT
RIGHT_DRIVE_WHEEL = 1                                                 # WheelID 1 = RIGHT

# --- Irrlicht visualization (Initialize first, then scene elements, NO grid) ---
vis = chronoirr.ChVisualSystemIrrlicht()                              # Irrlicht renderer
vis.AttachSystem(system)                                              # bind the simulation system
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)                     # Z-up camera convention
vis.SetWindowSize(1280, 720)                                          # window resolution
vis.SetWindowTitle("TurtleBot - Rigid terrain")                      # window title
vis.Initialize()                                                     # create the device first
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))     # PyChrono logo
vis.AddSkyBox()                                                      # sky box
vis.AddCamera(chrono.ChVector3d(0, 2.5, 1.5), chrono.ChVector3d(0, 0, 0.2))  # eye, look-at
vis.AddTypicalLights()                                               # standard lighting
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5), chrono.ChVector3d(0, 0, 0.5),
                       3, 4, 10, 40, 512)                            # shadow-casting light

# --- Simulation loop: differential-drive turns commanded at specific times ---
time_step = 2e-3                                                      # TurtleBot uses 2e-3
sim_end = 10.0                                                        # total simulated time
render_fps = 50.0                                                     # review-frame cadence
render_every = max(1, round(1.0 / (render_fps * time_step)))         # untagged cadence constant
time = 0.0                                                            # simulated clock
while vis.Run() and time < sim_end:
    vis.BeginScene()                                                 # start frame
    vis.Render()                                                     # draw scene
    vis.EndScene()                                                   # finish frame
    for _ in range(render_every):
        # command differential-drive turns at specific times (rad/s per wheel)
        if abs(time - 1.0) < 1e-3:                                   # at t = 1 s: pivot LEFT
            robot_tb.SetMotorSpeed(0, LEFT_DRIVE_WHEEL)              # left wheel stops
            robot_tb.SetMotorSpeed(-math.pi, RIGHT_DRIVE_WHEEL)     # right wheel drives
        if abs(time - 2.0) < 1e-3:                                   # at t = 2 s: pivot RIGHT
            robot_tb.SetMotorSpeed(-math.pi, LEFT_DRIVE_WHEEL)      # left wheel drives
            robot_tb.SetMotorSpeed(0, RIGHT_DRIVE_WHEEL)            # right wheel stops
        system.DoStepDynamics(time_step)                            # advance one step (NO Update on TurtleBot)
        time += time_step                                          # advance clock
        if time >= sim_end:
            break
