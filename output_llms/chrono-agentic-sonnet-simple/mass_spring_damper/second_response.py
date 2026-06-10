import os
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# Custom spring-force functor using ForceFunctor
class MySpringForce(chrono.ForceFunctor):
    def __init__(self):
        chrono.ForceFunctor.__init__(self)                             # call base ctor
        self.spring_coef = 50                                          # spring coefficient
        self.damping_coef = 1                                          # damping coefficient

    def evaluate(self, time, rest_length, length, vel, link):          # evaluate spring force
        force = -self.spring_coef * (length - rest_length) - self.damping_coef * vel
        return force                                                   # return scalar force

# System setup
sys = chrono.ChSystemNSC()                                            # NSC for spring-mass
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))      # Y-up gravity

# Ground body
ground = chrono.ChBody()                                               # create ground
ground.SetFixed(True)                                                  # immovable
sys.AddBody(ground)                                                    # add to system

# Visual sphere on ground at (0, 0, 0) for body_1 attachment
sph_1 = chrono.ChVisualShapeSphere(0.1)                               # visual sphere for spring_1 anchor
ground.AddVisualShape(sph_1, chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))   # attach at origin

# Visual sphere on ground at (1, 0, 0) for body_2 attachment
sph_2 = chrono.ChVisualShapeSphere(0.1)                               # visual sphere for spring_2 anchor
ground.AddVisualShape(sph_2, chrono.ChFramed(chrono.ChVector3d(1, 0, 0)))   # attach at x=1

# Body 1 — hanging mass for spring_1
body_1 = chrono.ChBody()                                               # first hanging body
body_1.SetMass(1.0)                                                    # 1 kg mass
body_1.SetInertiaXX(chrono.ChVector3d(1, 1, 1))                       # unit inertia
body_1.SetPos(chrono.ChVector3d(0, -3, 0))                            # initial position below anchor
sys.AddBody(body_1)                                                    # add to system

sph_body_1 = chrono.ChVisualShapeSphere(0.3)                          # visual sphere for body_1
body_1.AddVisualShape(sph_body_1)                                      # attach shape

# Body 2 — hanging mass for spring_2 (mirrors body_1 setup)
body_2 = chrono.ChBody()                                               # second hanging body
body_2.SetMass(1.0)                                                    # 1 kg mass (mirrors body_1)
body_2.SetInertiaXX(chrono.ChVector3d(1, 1, 1))                       # unit inertia
body_2.SetPos(chrono.ChVector3d(1, -3, 0))                            # initial position below sph_2 anchor
sys.AddBody(body_2)                                                    # add to system

sph_body_2 = chrono.ChVisualShapeSphere(0.3)                          # visual sphere for body_2
body_2.AddVisualShape(sph_body_2)                                      # attach shape

# Spring 1: connects body_1 to ground using direct spring and damping coefficients
spring_1 = chrono.ChLinkTSDA()                                         # translational spring-damper-actuator
spring_1.Initialize(body_1, ground, True,
                    chrono.ChVector3d(0, 0, 0),                        # attachment on body_1 (local)
                    chrono.ChVector3d(0, 0, 0))                        # attachment on ground (local origin)
spring_1.SetRestLength(1.5)                                            # rest length
spring_1.SetSpringCoefficient(50)                                      # direct spring coefficient
spring_1.SetDampingCoefficient(1)                                      # direct damping coefficient
sys.AddLink(spring_1)                                                  # add spring to system

spring_1.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 80, 15))     # spring coil visual

# Spring 2: connects body_2 to ground using custom force functor MySpringForce
spring_2 = chrono.ChLinkTSDA()                                         # second spring using custom functor
spring_2.Initialize(body_2, ground, True,
                    chrono.ChVector3d(0, 0, 0),                        # attachment on body_2 (local)
                    chrono.ChVector3d(1, 0, 0))                        # attachment on ground at (1,0,0)
spring_2.SetRestLength(1.5)                                            # rest length matches spring_1
spring_force_functor = MySpringForce()                                 # instantiate custom functor
spring_2.RegisterForceFunctor(spring_force_functor)                    # register custom force functor
sys.AddLink(spring_2)                                                  # add spring to system

spring_2.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 80, 15))     # spring coil visual

# Irrlicht visualization (Initialize FIRST, then add scene elements)
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)                                                  # attach physics system
vis.SetWindowSize(1280, 720)                                           # window dimensions
vis.SetWindowTitle("Mass-Spring-Damper with Custom Force Functor")     # window title
vis.Initialize()                                                       # Initialize FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))       # logo after Initialize
vis.AddSkyBox()                                                        # skybox after Initialize
vis.AddCamera(chrono.ChVector3d(0, 0, 10), chrono.ChVector3d(0, -2, 0))  # camera after Initialize
vis.AddTypicalLights()                                                 # lights after Initialize

time_step = 1e-3                                                       # physics time step [s]
sim_end = 10.0                                                         # simulation end time [s]
render_fps = 50.0                                                      # render frame rate [fps]
render_every = max(1, round(1.0 / (render_fps * time_step)))           # render cadence (untagged)

log_info = True                                                        # one-shot log flag (scored)
while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)                                  # advance physics
        if log_info and sys.GetChTime() > 1:                           # one-shot log after 1 s settling
            pos1 = body_1.GetPos()                                     # position of body_1
            print("t = ", sys.GetChTime())
            print("     ", pos1.x, "  ", pos1.y)
            vel1 = body_1.GetPosDt()                                   # linear velocity of body_1
            print("     ", vel1.x, "  ", vel1.y)
            log_info = False                                           # disable further logging
        if sys.GetChTime() >= sim_end:
            break
