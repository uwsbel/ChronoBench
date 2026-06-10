import os
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

sys = chrono.ChSystemNSC()                                            # NSC rigid-body system
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))      # Y-up gravity

rest_length = 1.5                                                      # spring free length
spring_k = 50                                                          # spring stiffness
damping_c = 1                                                          # damping coefficient

ground = chrono.ChBody()                                              # fixed anchor for the chain
ground.SetFixed(True)                                                 # ground does not move
ground.SetPos(chrono.ChVector3d(0, 0, 0))                            # anchor at origin
sys.AddBody(ground)

ground.AddVisualShape(chrono.ChVisualShapeBox(0.4, 0.1, 0.4),         # small slab marking the anchor
                      chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))

body_1 = chrono.ChBody()                                              # first hanging mass
body_1.SetMass(1)                                                     # mass of body_1
body_1.SetInertiaXX(chrono.ChVector3d(1, 1, 1))                       # diagonal inertia
body_1.SetPos(chrono.ChVector3d(0, -1.5, 0))                          # one rest length below ground
sys.AddBody(body_1)
body_1.AddVisualShape(chrono.ChVisualShapeBox(0.3, 0.3, 0.3))         # cube marker

body_2 = chrono.ChBody()                                              # second hanging mass
body_2.SetMass(1)                                                     # mass of body_2
body_2.SetInertiaXX(chrono.ChVector3d(1, 1, 1))                       # diagonal inertia
body_2.SetPos(chrono.ChVector3d(0, -3.0, 0))                          # below body_1
sys.AddBody(body_2)
body_2.AddVisualShape(chrono.ChVisualShapeBox(0.3, 0.3, 0.3))         # cube marker

body_3 = chrono.ChBody()                                              # third hanging mass
body_3.SetMass(1)                                                     # mass of body_3
body_3.SetInertiaXX(chrono.ChVector3d(1, 1, 1))                       # diagonal inertia
body_3.SetPos(chrono.ChVector3d(0, -4.5, 0))                          # below body_2
sys.AddBody(body_3)
body_3.AddVisualShape(chrono.ChVisualShapeBox(0.3, 0.3, 0.3))         # cube marker

spring_1 = chrono.ChLinkTSDA()                                        # ground -> body_1 spring
spring_1.Initialize(body_1, ground, True,                             # body-local attachment points
                    chrono.ChVector3d(0, 0, 0),
                    chrono.ChVector3d(0, 0, 0))
spring_1.SetRestLength(rest_length)                                  # free length
spring_1.SetSpringCoefficient(spring_k)                             # stiffness
spring_1.SetDampingCoefficient(damping_c)                           # damping
sys.AddLink(spring_1)
spring_1.AddVisualShape(chrono.ChVisualShapeSpring(0.08, 80, 15))    # coil visual on the link

spring_2 = chrono.ChLinkTSDA()                                        # body_1 -> body_2 spring
spring_2.Initialize(body_2, body_1, True,                             # body-local attachment points
                    chrono.ChVector3d(0, 0, 0),
                    chrono.ChVector3d(0, 0, 0))
spring_2.SetRestLength(rest_length)                                  # free length
spring_2.SetSpringCoefficient(spring_k)                             # stiffness
spring_2.SetDampingCoefficient(damping_c)                           # damping
sys.AddLink(spring_2)
spring_2.AddVisualShape(chrono.ChVisualShapeSpring(0.08, 80, 15))    # coil visual on the link

spring_3 = chrono.ChLinkTSDA()                                        # body_2 -> body_3 spring
spring_3.Initialize(body_3, body_2, True,                             # body-local attachment points
                    chrono.ChVector3d(0, 0, 0),
                    chrono.ChVector3d(0, 0, 0))
spring_3.SetRestLength(rest_length)                                  # free length
spring_3.SetSpringCoefficient(spring_k)                             # stiffness
spring_3.SetDampingCoefficient(damping_c)                           # damping
sys.AddLink(spring_3)
spring_3.AddVisualShape(chrono.ChVisualShapeSpring(0.08, 80, 15))    # coil visual on the link

vis = chronoirr.ChVisualSystemIrrlicht()                             # Irrlicht render window
vis.AttachSystem(sys)                                                # bind the physical system
vis.SetWindowSize(1280, 720)                                         # window pixels
vis.SetWindowTitle("Mass-Spring-Damper Chain")                      # window title
vis.Initialize()                                                     # create device FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))    # PyChrono logo
vis.AddSkyBox()                                                      # sky background
vis.AddCamera(chrono.ChVector3d(0, -2.5, 6), chrono.ChVector3d(0, -2.5, 0))  # view the chain
vis.AddTypicalLights()                                              # standard lighting

time_step = 1e-3                                                     # integration step
sim_end = 10.0                                                       # simulation end time
render_fps = 50.0                                                   # frames per second for review
render_every = max(1, round(1.0 / (render_fps * time_step)))        # physics steps per frame
while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)
        if sys.GetChTime() >= sim_end:
            break
