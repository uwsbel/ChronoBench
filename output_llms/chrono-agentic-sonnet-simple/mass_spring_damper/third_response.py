import os
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# simulation parameters
time_step = 1e-3                                                   # physics step [s]
sim_end = 10.0                                                     # simulation duration [s]
render_fps = 50.0                                                  # review video frame rate

# create the physical system (NSC, Y-up, no gravity for pure spring-mass)
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))      # no gravity — spring restores

# spring / damper parameters
spring_k = 50.0                                                    # spring stiffness [N/m]
damping_c = 1.0                                                    # damping coefficient [N·s/m]
rest_len = 1.5                                                     # rest length [m]

# body masses and inertia
body_mass = 1.0                                                    # mass of each body [kg]
body_inertia = 0.1                                                 # inertia [kg·m²]

# fixed ground body
ground = chrono.ChBody()
ground.SetFixed(True)                                              # immovable ground
ground.SetPos(chrono.ChVector3d(0, 0, 0))                         # at world origin
sys.AddBody(ground)
ground_shape = chrono.ChVisualShapeBox(0.5, 0.5, 0.5)             # small box to visualize ground
ground.AddVisualShape(ground_shape)

# body_1: first mass, connected to ground via spring
body_1 = chrono.ChBody()
body_1.SetMass(body_mass)                                          # mass [kg]
body_1.SetInertiaXX(chrono.ChVector3d(body_inertia, body_inertia, body_inertia))
body_1.SetPos(chrono.ChVector3d(-3.0, 0, 0))                      # initial displaced position
sys.AddBody(body_1)
body_1_shape = chrono.ChVisualShapeSphere(0.2)                    # sphere visual
body_1.AddVisualShape(body_1_shape)

# body_2: second mass, connected to body_1 via spring
body_2 = chrono.ChBody()
body_2.SetMass(body_mass)                                          # mass [kg]
body_2.SetInertiaXX(chrono.ChVector3d(body_inertia, body_inertia, body_inertia))
body_2.SetPos(chrono.ChVector3d(-1.5, 0, 0))                      # initial position
sys.AddBody(body_2)
body_2_shape = chrono.ChVisualShapeSphere(0.2)                    # sphere visual
body_2.AddVisualShape(body_2_shape)

# body_3: third mass, connected to body_2 via spring
body_3 = chrono.ChBody()
body_3.SetMass(body_mass)                                          # mass [kg]
body_3.SetInertiaXX(chrono.ChVector3d(body_inertia, body_inertia, body_inertia))
body_3.SetPos(chrono.ChVector3d(0.0, 0, 0))                       # initial position
sys.AddBody(body_3)
body_3_shape = chrono.ChVisualShapeSphere(0.2)                    # sphere visual
body_3.AddVisualShape(body_3_shape)

# spring between ground and body_1
spring_1 = chrono.ChLinkTSDA()
spring_1.Initialize(
    ground, body_1, True,
    chrono.ChVector3d(0, 0, 0),                                    # attachment on ground (local)
    chrono.ChVector3d(0, 0, 0)                                     # attachment on body_1 (local)
)
spring_1.SetRestLength(rest_len)                                   # rest length [m]
spring_1.SetSpringCoefficient(spring_k)                            # stiffness [N/m]
spring_1.SetDampingCoefficient(damping_c)                          # damping [N·s/m]
sys.AddLink(spring_1)
spring_1.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 80, 15)) # spring coil visual

# spring between body_1 and body_2
spring_2 = chrono.ChLinkTSDA()
spring_2.Initialize(
    body_1, body_2, True,
    chrono.ChVector3d(0, 0, 0),                                    # attachment on body_1 (local)
    chrono.ChVector3d(0, 0, 0)                                     # attachment on body_2 (local)
)
spring_2.SetRestLength(rest_len)                                   # rest length [m]
spring_2.SetSpringCoefficient(spring_k)                            # stiffness [N/m]
spring_2.SetDampingCoefficient(damping_c)                          # damping [N·s/m]
sys.AddLink(spring_2)
spring_2.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 80, 15)) # spring coil visual

# spring between body_2 and body_3
spring_3 = chrono.ChLinkTSDA()
spring_3.Initialize(
    body_2, body_3, True,
    chrono.ChVector3d(0, 0, 0),                                    # attachment on body_2 (local)
    chrono.ChVector3d(0, 0, 0)                                     # attachment on body_3 (local)
)
spring_3.SetRestLength(rest_len)                                   # rest length [m]
spring_3.SetSpringCoefficient(spring_k)                            # stiffness [N/m]
spring_3.SetDampingCoefficient(damping_c)                          # damping [N·s/m]
sys.AddLink(spring_3)
spring_3.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 80, 15)) # spring coil visual

# Irrlicht visualization — Initialize() FIRST, then add scene elements
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)                                              # bind to the system
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)                 # Z-up camera
vis.SetWindowSize(1280, 720)                                      # window size [px]
vis.SetWindowTitle("Mass-Spring-Damper Chain: 3 Bodies")
vis.Initialize()                                                   # create Irrlicht device
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))  # logo after init
vis.AddSkyBox()                                                    # sky box
vis.AddCamera(
    chrono.ChVector3d(0, -10, 3),                                 # camera eye
    chrono.ChVector3d(0, 0, 0)                                    # camera target
)
vis.AddTypicalLights()                                             # standard lighting

# render cadence constant (untagged — truth-shaped loop needs it)
render_every = max(1, round(1.0 / (render_fps * time_step)))      # steps per rendered frame

# review-only: recording setup

# main simulation loop
while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)                              # advance one step
        if sys.GetChTime() >= sim_end:
            break
