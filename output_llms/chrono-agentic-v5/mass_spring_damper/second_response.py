"""Mass-spring-damper demo with two ChLinkTSDA spring-dampers (PyChrono / NSC).

Models two unit-mass boxes suspended from a fixed ground by translational
spring-dampers built two different ways:
  * spring_1 — a default linear ChLinkTSDA using direct spring/damping
    coefficients between body_1 and the ground.
  * spring_2 — a ChLinkTSDA whose force is supplied by a custom
    chrono.ForceFunctor (MySpringForce) reproducing the same linear law with
    spring_coef = 50, damping_coef = 1 between body_2 and the ground.
The ground carries two visualization spheres (sph_1, sph_2) marking the spring
anchor points. With zero gravity each box oscillates about the spring rest
length and decays toward equilibrium under the damping term. System type: NSC,
pure jointed MBS (no contact), so no collision system is configured.
"""

import os
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# === Parameters === spring/damper law + simulation control constants
rest_length = 1.5
spring_coef = 50
damping_coef = 1

time_step = 1e-3
sim_end = 10.0
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))   # precomputed once


# === Custom force functor === evaluates a linear spring-damper force for a TSDA
class MySpringForce(chrono.ForceFunctor):
    def __init__(self):
        super(MySpringForce, self).__init__()

    def evaluate(self, time, rest_length, length, vel, link):
        # Linear spring-damper: restoring + viscous term (positive vel = extending)
        force = -spring_coef * (length - rest_length) - damping_coef * vel
        return force


# === System & gravity === NSC system, zero gravity so springs drive the motion
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

# === Bodies === fixed ground with two anchor spheres, plus two suspended boxes
ground = chrono.ChBody()
sys.AddBody(ground)
ground.SetFixed(True)
ground.EnableCollision(False)

sph_1 = chrono.ChVisualShapeSphere(0.1)
ground.AddVisualShape(sph_1, chrono.ChFramed(chrono.ChVector3d(-1, 0, 0)))

sph_2 = chrono.ChVisualShapeSphere(0.1)
ground.AddVisualShape(sph_2, chrono.ChFramed(chrono.ChVector3d(1, 0, 0)))

body_1 = chrono.ChBody()
sys.AddBody(body_1)
body_1.SetPos(chrono.ChVector3d(-1, -3, 0))
body_1.SetFixed(False)
body_1.EnableCollision(False)
body_1.SetMass(1)
body_1.SetInertiaXX(chrono.ChVector3d(1, 1, 1))

box_1 = chrono.ChVisualShapeBox(1, 1, 1)
box_1.SetColor(chrono.ChColor(0.6, 0, 0))
body_1.AddVisualShape(box_1)

body_2 = chrono.ChBody()
sys.AddBody(body_2)
body_2.SetPos(chrono.ChVector3d(1, -3, 0))
body_2.SetFixed(False)
body_2.EnableCollision(False)
body_2.SetMass(1)
body_2.SetInertiaXX(chrono.ChVector3d(1, 1, 1))

box_2 = chrono.ChVisualShapeBox(1, 1, 1)
box_2.SetColor(chrono.ChColor(0, 0, 0.6))
body_2.AddVisualShape(box_2)

# === Spring-dampers === direct-coefficient spring vs custom-functor spring
spring_1 = chrono.ChLinkTSDA()
spring_1.Initialize(body_1, ground, True, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(-1, 0, 0))
spring_1.SetRestLength(rest_length)
spring_1.SetSpringCoefficient(spring_coef)
spring_1.SetDampingCoefficient(damping_coef)
sys.AddLink(spring_1)
spring_1.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 80, 15))

force = MySpringForce()                       # cache: functor built once, reused every step
spring_2 = chrono.ChLinkTSDA()
spring_2.Initialize(body_2, ground, True, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(1, 0, 0))
spring_2.SetRestLength(rest_length)
spring_2.RegisterForceFunctor(force)
sys.AddLink(spring_2)
spring_2.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 80, 15))

# === Visualization === full Irrlicht scene: window + sky + camera + lights
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("ChLinkTSDA demo")
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 0, 6))
vis.AddTypicalLights()

# === Main loop === step the spring-damper system, capture review frames + data
os.makedirs("cam", exist_ok=True)              # guard against missing output dir

frame = 0
try:
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            sys.DoStepDynamics(time_step)
            if sys.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:      # solver divergence / bad state
    import traceback; traceback.print_exc()
    raise
finally:
    pass
