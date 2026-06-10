import os
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

sys = chrono.ChSystemNSC()                                            # NSC rigid-body system (truth uses NSC)
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))          # spring-only dynamics, gravity disabled

rest_length = 1.0                                                     # spring natural length [m]
spring_k = 50.0                                                       # spring stiffness [N/m]
damping_c = 0.2                                                       # light spring damping -> sustained visible oscillation [N.s/m]
mass_value = 1.0                                                      # mass of each moving body [kg]
init_disp = 1.0                                                       # initial stretch applied to each mass [m]

ground = chrono.ChBody()                                             # fixed anchor body for the first spring
ground.SetFixed(True)                                               # ground does not move
ground.SetPos(chrono.ChVector3d(0, 0, 0))                          # anchor at the origin
sys.AddBody(ground)                                                 # register the ground

ground_vis = chrono.ChVisualShapeSphere(0.1)                         # small marker at the anchor
ground_vis.SetColor(chrono.ChColor(0.2, 0.2, 0.2))                  # dark grey anchor
ground.AddVisualShape(ground_vis)                                  # attach the anchor marker

body_1 = chrono.ChBody()                                            # first mass in the chain
body_1.SetMass(mass_value)                                          # 1 kg
body_1.SetInertiaXX(chrono.ChVector3d(1, 1, 1))                    # diagonal inertia
body_1.SetPos(chrono.ChVector3d(rest_length + init_disp, 0, 0))    # placed past rest length -> initial stretch
sys.AddBody(body_1)                                                # register body_1

box_1 = chrono.ChVisualShapeBox(0.3, 0.3, 0.3)                       # cube visual for body_1
box_1.SetColor(chrono.ChColor(0.6, 0.2, 0.2))                       # red mass
body_1.AddVisualShape(box_1)                                        # attach the visual

body_2 = chrono.ChBody()                                            # second mass in the chain
body_2.SetMass(mass_value)                                          # 1 kg
body_2.SetInertiaXX(chrono.ChVector3d(1, 1, 1))                    # diagonal inertia
body_2.SetPos(chrono.ChVector3d(2 * rest_length + init_disp, 0, 0))  # one rest length beyond body_1 plus stretch
sys.AddBody(body_2)                                                # register body_2

box_2 = chrono.ChVisualShapeBox(0.3, 0.3, 0.3)                       # cube visual for body_2
box_2.SetColor(chrono.ChColor(0.2, 0.6, 0.2))                       # green mass
body_2.AddVisualShape(box_2)                                        # attach the visual

body_3 = chrono.ChBody()                                            # third mass in the chain
body_3.SetMass(mass_value)                                          # 1 kg
body_3.SetInertiaXX(chrono.ChVector3d(1, 1, 1))                    # diagonal inertia
body_3.SetPos(chrono.ChVector3d(3 * rest_length + init_disp, 0, 0))  # one rest length beyond body_2 plus stretch
sys.AddBody(body_3)                                                # register body_3

box_3 = chrono.ChVisualShapeBox(0.3, 0.3, 0.3)                       # cube visual for body_3
box_3.SetColor(chrono.ChColor(0.2, 0.2, 0.6))                       # blue mass
body_3.AddVisualShape(box_3)                                        # attach the visual

spring_1 = chrono.ChLinkTSDA()                                       # spring-damper ground -> body_1
spring_1.Initialize(body_1, ground, True,                            # body-local attachment frames
                    chrono.ChVector3d(0, 0, 0),                      # attach at body_1 center
                    chrono.ChVector3d(0, 0, 0))                      # attach at ground anchor
spring_1.SetRestLength(rest_length)                                  # natural length
spring_1.SetSpringCoefficient(spring_k)                              # stiffness k
spring_1.SetDampingCoefficient(damping_c)                            # damping c
sys.AddLink(spring_1)                                               # register the spring
spring_1.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 80, 15))    # coil radius, resolution, turns

spring_2 = chrono.ChLinkTSDA()                                       # spring-damper body_1 -> body_2
spring_2.Initialize(body_2, body_1, True,                            # body-local attachment frames
                    chrono.ChVector3d(0, 0, 0),                      # attach at body_2 center
                    chrono.ChVector3d(0, 0, 0))                      # attach at body_1 center
spring_2.SetRestLength(rest_length)                                  # natural length
spring_2.SetSpringCoefficient(spring_k)                              # stiffness k
spring_2.SetDampingCoefficient(damping_c)                            # damping c
sys.AddLink(spring_2)                                               # register the spring
spring_2.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 80, 15))    # coil radius, resolution, turns

spring_3 = chrono.ChLinkTSDA()                                       # spring-damper body_2 -> body_3
spring_3.Initialize(body_3, body_2, True,                            # body-local attachment frames
                    chrono.ChVector3d(0, 0, 0),                      # attach at body_3 center
                    chrono.ChVector3d(0, 0, 0))                      # attach at body_2 center
spring_3.SetRestLength(rest_length)                                  # natural length
spring_3.SetSpringCoefficient(spring_k)                              # stiffness k
spring_3.SetDampingCoefficient(damping_c)                            # damping c
sys.AddLink(spring_3)                                               # register the spring
spring_3.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 80, 15))    # coil radius, resolution, turns

vis = chronoirr.ChVisualSystemIrrlicht()                            # Irrlicht render window
vis.AttachSystem(sys)                                              # bind the physical system
vis.SetWindowSize(1280, 720)                                       # window resolution
vis.SetWindowTitle("Mass-Spring-Damper Chain")                     # window title
vis.Initialize()                                                  # create the device first
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))   # PyChrono logo
vis.AddSkyBox()                                                   # sky background
vis.AddCamera(chrono.ChVector3d(2, -4, 2), chrono.ChVector3d(2, 0, 0))  # eye and look-at the chain
vis.AddTypicalLights()                                            # standard two-light setup

time_step = 1e-3                                                    # integration step [s]
sim_end = 10.0                                                     # total simulated time [s]
render_fps = 50.0                                                  # frames per second for the render
render_every = max(1, round(1.0 / (render_fps * time_step)))      # physics steps between rendered frames
log_info = True                                                    # fire the physics log once
while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()                                               # start the frame
    vis.Render()                                                   # draw the scene
    vis.EndScene()                                                 # finish the frame
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)                              # advance one step
        if log_info and sys.GetChTime() > 1:                       # after 1 s of motion
            p1 = body_1.GetPos()                                   # position of body_1
            p2 = body_2.GetPos()                                   # position of body_2
            p3 = body_3.GetPos()                                   # position of body_3
            print("t = ", sys.GetChTime())
            print("     ", p1.x, "  ", p2.x, "  ", p3.x)
            v1 = body_1.GetPosDt()                                 # velocity of body_1
            print("     ", v1.x)
            log_info = False                                       # disable further logging
        if sys.GetChTime() >= sim_end:
            break
