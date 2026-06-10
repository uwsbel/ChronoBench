import os
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

sys = chrono.ChSystemNSC()                                            # rigid-body NSC system
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))      # Y-up gravity, g = 9.81

rest_length = 1.5                                                      # spring free length [m]
spring_k = 50.0                                                        # default linear stiffness [N/m]
damping_c = 1.0                                                        # default linear damping [N·s/m]
mass = 1.0                                                             # suspended mass [kg]

ground = chrono.ChBody()                                              # fixed support at the top
ground.SetFixed(True)                                                 # immovable anchor
ground.EnableCollision(False)                                         # pure jointed/spring MBS, no contact
ground.SetPos(chrono.ChVector3d(0, 0, 0))                            # anchor at the world origin
sys.AddBody(ground)                                                   # register the ground body

ground_marker = chrono.ChVisualShapeBox(0.2, 0.2, 0.2)               # small cube marking the anchor
ground_marker.SetColor(chrono.ChColor(0.4, 0.4, 0.4))                # grey support block
ground.AddVisualShape(ground_marker)                                 # attach the marker visual

body = chrono.ChBody()                                               # the oscillating mass
body.SetMass(mass)                                                   # set its mass
body.SetInertiaXX(chrono.ChVector3d(0.01, 0.01, 0.01))               # small isotropic inertia
body.EnableCollision(False)                                          # no contact for the mass
body.SetPos(chrono.ChVector3d(0, -rest_length, 0))                  # start hanging at the rest length
sys.AddBody(body)                                                    # register the mass body

box = chrono.ChVisualShapeBox(0.4, 0.4, 0.4)                         # cube visual for the mass
box.SetColor(chrono.ChColor(0.2, 0.5, 0.8))                          # blue mass block
body.AddVisualShape(box)                                             # attach the mass visual

spring = chrono.ChLinkTSDA()                                         # translational spring-damper
spring.Initialize(body, ground, True,                               # connect mass to ground (body-local frames)
                  chrono.ChVector3d(0, 0, 0),                       # attach point on the mass (its center)
                  chrono.ChVector3d(0, 0, 0))                       # attach point on the ground (origin)
spring.SetRestLength(rest_length)                                   # natural length of the spring
spring.SetSpringCoefficient(spring_k)                               # linear stiffness coefficient
spring.SetDampingCoefficient(damping_c)                             # linear damping coefficient
sys.AddLink(spring)                                                  # register the spring-damper link

spring.AddVisualShape(chrono.ChVisualShapeSpring(0.08, 80, 15))     # coil radius, resolution, turns

vis = chronoirr.ChVisualSystemIrrlicht()                            # create the Irrlicht renderer
vis.AttachSystem(sys)                                                # bind it to the physical system
vis.SetWindowSize(1280, 720)                                        # window resolution
vis.SetWindowTitle("Mass-Spring-Damper")                            # window title
vis.Initialize()                                                    # build the device FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))    # PyChrono logo overlay
vis.AddSkyBox()                                                      # standard sky box
vis.AddCamera(chrono.ChVector3d(0, -1.0, 4), chrono.ChVector3d(0, -1.0, 0))  # eye / look-at the mass
vis.AddTypicalLights()                                              # standard two-light setup

time_step = 1e-3                                                     # integration step [s]
sim_end = 10.0                                                       # simulation duration [s]
render_fps = 50.0                                                    # target frames per second
render_every = max(1, round(1.0 / (render_fps * time_step)))        # physics steps per rendered frame
while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()                                               # open the render frame
    vis.Render()                                                   # draw the scene
    vis.EndScene()                                                 # finish the render frame
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)                              # advance the dynamics one step
        if sys.GetChTime() >= sim_end:
            break
