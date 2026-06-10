import os
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

sys = chrono.ChSystemNSC()                                            # NSC system (no contact in this scene)
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))          # gravity OFF — spring force only

rest_length = 1.5                                                     # spring rest length
spring_coef = 50                                                      # spring stiffness k
damping_coef = 1                                                      # damping c

ground = chrono.ChBody()                                             # fixed anchor body
ground.SetFixed(True)
ground.EnableCollision(False)
sys.AddBody(ground)
sphere_1 = chrono.ChVisualShapeSphere(0.1)                           # top anchor marker
ground.AddVisualShape(sphere_1, chrono.ChFramed(chrono.ChVector3d(-1, 0, 0)))

body_1 = chrono.ChBody()                                             # first mass in the chain
body_1.SetPos(chrono.ChVector3d(-1, -3, 0))
body_1.SetMass(1)
body_1.SetInertiaXX(chrono.ChVector3d(1, 1, 1))
body_1.EnableCollision(False)
sys.AddBody(body_1)
box_1 = chrono.ChVisualShapeBox(1, 1, 1)                             # 1x1x1 cube
box_1.SetColor(chrono.ChColor(0.6, 0, 0))
body_1.AddVisualShape(box_1)

body_2 = chrono.ChBody()                                             # second mass, hangs below body_1
body_2.SetPos(chrono.ChVector3d(-1, -6, 0))
body_2.SetMass(1)
body_2.SetInertiaXX(chrono.ChVector3d(1, 1, 1))
body_2.EnableCollision(False)
sys.AddBody(body_2)
box_2 = chrono.ChVisualShapeBox(1, 1, 1)
box_2.SetColor(chrono.ChColor(0, 0.6, 0))
body_2.AddVisualShape(box_2)

body_3 = chrono.ChBody()                                             # third mass, hangs below body_2
body_3.SetPos(chrono.ChVector3d(-1, -9, 0))
body_3.SetMass(1)
body_3.SetInertiaXX(chrono.ChVector3d(1, 1, 1))
body_3.EnableCollision(False)
sys.AddBody(body_3)
box_3 = chrono.ChVisualShapeBox(1, 1, 1)
box_3.SetColor(chrono.ChColor(0, 0, 0.6))
body_3.AddVisualShape(box_3)

spring_1 = chrono.ChLinkTSDA()                                       # ground anchor -> body_1
spring_1.Initialize(body_1, ground, True, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(-1, 0, 0))
spring_1.SetRestLength(rest_length)
spring_1.SetSpringCoefficient(spring_coef)
spring_1.SetDampingCoefficient(damping_coef)
sys.AddLink(spring_1)
spring_1.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 80, 15))    # coil viz on the link

spring_2 = chrono.ChLinkTSDA()                                       # body_1 -> body_2
spring_2.Initialize(body_2, body_1, True, chrono.VNULL, chrono.VNULL)
spring_2.SetRestLength(rest_length)
spring_2.SetSpringCoefficient(spring_coef)
spring_2.SetDampingCoefficient(damping_coef)
sys.AddLink(spring_2)
spring_2.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 80, 15))

spring_3 = chrono.ChLinkTSDA()                                       # body_2 -> body_3
spring_3.Initialize(body_3, body_2, True, chrono.VNULL, chrono.VNULL)
spring_3.SetRestLength(rest_length)
spring_3.SetSpringCoefficient(spring_coef)
spring_3.SetDampingCoefficient(damping_coef)
sys.AddLink(spring_3)
spring_3.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 80, 15))

vis = chronoirr.ChVisualSystemIrrlicht()                            # Irrlicht window
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Mass-spring-damper chain")
vis.Initialize()                                                    # device first
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, -3, 6))                          # look at the hanging chain
vis.AddTypicalLights()

time_step = 1e-3                                                    # integration step
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))        # untagged cadence constant
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)
