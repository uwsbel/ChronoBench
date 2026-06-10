import os
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

spring_coef = 50                                  # spring stiffness coefficient
damping_coef = 1                                  # spring damping coefficient
rest_length = 1.5                                 # spring free length [m]


class MySpringForce(chrono.ForceFunctor):         # custom spring-force evaluator
    def __init__(self):
        super().__init__()                        # MUST call base ctor
    def evaluate(self, time, rest_length, length, vel, link):   # 9.0.0 override name
        force = -spring_coef * (length - rest_length) - damping_coef * vel   # F = -k*dx - c*v
        return force


sys = chrono.ChSystemNSC()                        # rigid-body system, default solver
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))   # gravity disabled for spring demo

ground = chrono.ChBody()                          # fixed reference body
ground.SetFixed(True)                             # ground is immovable
sys.AddBody(ground)

sph_1 = chrono.ChVisualShapeSphere(0.1)           # visual anchor sphere for spring_1
ground.AddVisualShape(sph_1, chrono.ChFramed(chrono.ChVector3d(-1, 0, 0)))   # left anchor
sph_2 = chrono.ChVisualShapeSphere(0.1)           # visual anchor sphere for spring_2
ground.AddVisualShape(sph_2, chrono.ChFramed(chrono.ChVector3d(1, 0, 0)))    # right anchor

body_1 = chrono.ChBody()                          # mass driven by spring_1
body_1.SetMass(1)                                 # body mass [kg]
body_1.SetInertiaXX(chrono.ChVector3d(1, 1, 1))   # diagonal inertia
body_1.SetPos(chrono.ChVector3d(-1, -3, 0))       # start below the left anchor
body_1.SetFixed(False)                            # dynamic body
sys.AddBody(body_1)
box_1 = chrono.ChVisualShapeBox(0.4, 0.4, 0.4)    # full-extent visual box
body_1.AddVisualShape(box_1)

body_2 = chrono.ChBody()                           # mass driven by spring_2
body_2.SetMass(1)                                  # body mass [kg]
body_2.SetInertiaXX(chrono.ChVector3d(1, 1, 1))    # diagonal inertia
body_2.SetPos(chrono.ChVector3d(1, -3, 0))         # start below the right anchor
body_2.SetFixed(False)                             # dynamic body
sys.AddBody(body_2)
box_2 = chrono.ChVisualShapeBox(0.4, 0.4, 0.4)     # full-extent visual box
body_2.AddVisualShape(box_2)

spring_1 = chrono.ChLinkTSDA()                      # spring with direct coefficients
spring_1.Initialize(body_1, ground, True,
                    chrono.ChVector3d(0, 0, 0),     # attach on body_1 origin
                    chrono.ChVector3d(-1, 0, 0))    # attach at left ground anchor
spring_1.SetRestLength(rest_length)                 # free length
spring_1.SetSpringCoefficient(spring_coef)          # k = 50
spring_1.SetDampingCoefficient(damping_coef)        # c = 1
sys.AddLink(spring_1)
spring_1.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 80, 15))   # coil visual

force_functor = MySpringForce()                     # custom force functor instance
spring_2 = chrono.ChLinkTSDA()                       # spring driven by the custom functor
spring_2.Initialize(body_2, ground, True,
                    chrono.ChVector3d(0, 0, 0),      # attach on body_2 origin
                    chrono.ChVector3d(1, 0, 0))      # attach at right ground anchor
spring_2.SetRestLength(rest_length)                  # free length
spring_2.RegisterForceFunctor(force_functor)         # use custom functor for force
sys.AddLink(spring_2)
spring_2.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 80, 15))   # coil visual

vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Mass-Spring-Damper")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 0, 6), chrono.ChVector3d(0, -2, 0))
vis.AddTypicalLights()

time_step = 1e-3                                     # integration step [s]
sim_end = 10.0                                       # stop time [s]
render_fps = 50.0                                    # frames per second
render_every = max(1, round(1.0 / (render_fps * time_step)))   # untagged cadence constant
while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)
        if sys.GetChTime() >= sim_end:
            break
