import os
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

rest_length = 1.5                                                     # spring natural length
spring_coef = 50                                                      # spring stiffness k
damping_coef = 1                                                      # damping c

# Custom force functor evaluating the spring-damper force from custom coefficients.
class MySpringForce(chrono.ForceFunctor):
    def __init__(self):
        chrono.ForceFunctor.__init__(self)                           # MUST call base ctor
    def evaluate(self, time, rest_length, length, vel, link):        # 9.0.0 override name
        force = -spring_coef * (length - rest_length) - damping_coef * vel  # linear spring-damper
        return force

sys = chrono.ChSystemNSC()                                           # NSC system (pure jointed MBS)
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))         # gravity OFF — spring is the only force

ground = chrono.ChBody()                                             # fixed anchor body
ground.SetFixed(True)
ground.EnableCollision(False)
sys.AddBody(ground)

sph_1 = chrono.ChVisualShapeSphere(0.1)                              # anchor marker for spring_1
ground.AddVisualShape(sph_1, chrono.ChFramed(chrono.ChVector3d(-1, 0, 0), chrono.QUNIT))
sph_2 = chrono.ChVisualShapeSphere(0.1)                              # anchor marker for spring_2 (mirrors sph_1)
ground.AddVisualShape(sph_2, chrono.ChFramed(chrono.ChVector3d(1, 0, 0), chrono.QUNIT))

body_1 = chrono.ChBody()                                             # first oscillating mass
body_1.SetPos(chrono.ChVector3d(-1, -3, 0))                          # hangs below the (-1,0,0) anchor
body_1.SetMass(1)
body_1.SetInertiaXX(chrono.ChVector3d(1, 1, 1))
body_1.EnableCollision(False)
sys.AddBody(body_1)
box_1 = chrono.ChVisualShapeBox(1, 1, 1)                             # 1x1x1 cube visual
box_1.SetColor(chrono.ChColor(0.6, 0, 0))
body_1.AddVisualShape(box_1)

body_2 = chrono.ChBody()                                             # second mass, mirrors body_1
body_2.SetPos(chrono.ChVector3d(1, -3, 0))                           # hangs below the (1,0,0) anchor
body_2.SetMass(1)
body_2.SetInertiaXX(chrono.ChVector3d(1, 1, 1))
body_2.EnableCollision(False)
sys.AddBody(body_2)
box_2 = chrono.ChVisualShapeBox(1, 1, 1)                             # 1x1x1 cube visual
box_2.SetColor(chrono.ChColor(0.6, 0, 0))
body_2.AddVisualShape(box_2)

# spring_1: direct spring + damping coefficients (existing setup, retained).
spring_1 = chrono.ChLinkTSDA()
spring_1.Initialize(body_1, ground, True, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(-1, 0, 0))  # body-relative endpoints
spring_1.SetRestLength(rest_length)
spring_1.SetSpringCoefficient(spring_coef)
spring_1.SetDampingCoefficient(damping_coef)
sys.AddLink(spring_1)
spring_1.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 80, 15))    # coil viz on the link

# spring_2: forces computed by the custom MySpringForce functor.
my_force = MySpringForce()                                           # functor instance kept alive
spring_2 = chrono.ChLinkTSDA()
spring_2.Initialize(body_2, ground, True, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(1, 0, 0))  # body-relative endpoints
spring_2.SetRestLength(rest_length)
spring_2.RegisterForceFunctor(my_force)                             # custom functor supplies the force
sys.AddLink(spring_2)
spring_2.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 80, 15))    # coil viz on the link

vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("ChLinkTSDA demo")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 0, 6))                            # face the springs from +Z
vis.AddTypicalLights()

time_step = 1e-3                                                     # integration step
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))         # untagged cadence constant
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)
