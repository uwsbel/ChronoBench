"""One-dimensional chained mass-spring-damper model in a PyChrono NSC system.

The scene contains a fixed anchor plus body_1, body_2, and body_3 constrained to
slide along the X axis. A TSDA spring-damper connects the anchor to body_1, with
additional springs between body_1 and body_2 and between body_2 and body_3. The
chain should oscillate along the guide axis while the added bodies remain visibly
connected by their compliant links.
"""

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr


# === Constants: define the 1D chain geometry and spring dynamics ===
TIME_STEP = 0.001
SIM_END = 6.0
STEPS_PER_FRAME = 1
SPHERE_RADIUS = 0.16
BODY_MASS = 1.0
BODY_INERTIA = chrono.ChVector3d(0.1, 0.1, 0.1)
SPRING_K = 70.0
DAMPING_C = 0.35
REST_LENGTH = 1.0
INITIAL_GAP = 1.45
INITIAL_SPEED = 1.1


# === System: NSC mechanics with gravity disabled for horizontal oscillator motion ===
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))


# === Bodies: fixed anchor plus three sliding masses ===
anchor = chrono.ChBody()
anchor.SetName("anchor")
anchor.SetFixed(True)
anchor.SetPos(chrono.ChVector3d(0, 0, 0))
anchor_shape = chrono.ChVisualShapeSphere(SPHERE_RADIUS)
anchor_shape.SetColor(chrono.ChColor(0.25, 0.25, 0.25))
anchor.AddVisualShape(anchor_shape)
sys.AddBody(anchor)


def make_mass(name, x_pos, speed):
    body = chrono.ChBody()
    body.SetName(name)
    body.SetMass(BODY_MASS)
    body.SetInertiaXX(BODY_INERTIA)
    body.SetPos(chrono.ChVector3d(x_pos, 0, 0))
    body.SetPosDt(chrono.ChVector3d(speed, 0, 0))
    body.EnableCollision(False)
    shape = chrono.ChVisualShapeSphere(SPHERE_RADIUS)
    shape.SetColor(chrono.ChColor(0.1, 0.45, 0.85))
    body.AddVisualShape(shape)
    sys.AddBody(body)
    return body


body_1 = make_mass("body_1", INITIAL_GAP, INITIAL_SPEED)
body_2 = make_mass("body_2", 2.0 * INITIAL_GAP, 0.0)
body_3 = make_mass("body_3", 3.0 * INITIAL_GAP, 0.0)
sliding_bodies = (body_1, body_2, body_3)  # cache: reused for guide setup and logging


# === Joints and springs: constrain each mass to X and connect the compliant chain ===
for slider in sliding_bodies:
    guide = chrono.ChLinkLockPrismatic()
    guide.Initialize(slider, anchor, chrono.ChFramed(slider.GetPos(), chrono.Q_ROTATE_Z_TO_X))
    sys.AddLink(guide)


def add_spring(name, body_a, body_b):
    spring = chrono.ChLinkTSDA()
    spring.SetName(name)
    spring.Initialize(
        body_a,
        body_b,
        True,
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
    )
    spring.SetRestLength(REST_LENGTH)
    spring.SetSpringCoefficient(SPRING_K)
    spring.SetDampingCoefficient(DAMPING_C)
    spring.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 80, 8))
    sys.AddLink(spring)
    return spring


spring_01 = add_spring("spring_anchor_body_1", anchor, body_1)
spring_12 = add_spring("spring_body_1_body_2", body_1, body_2)
spring_23 = add_spring("spring_body_2_body_3", body_2, body_3)


# === Visualization: Irrlicht window with camera, sky, logo, lights, and guide grid ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Mass Spring Damper Chain")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(2.0, -5.0, 2.0), chrono.ChVector3d(1.8, 0, 0))
vis.AddTypicalLights()
vis.AddGrid(
    0.5,
    0.5,
    12,
    6,
    chrono.ChCoordsysd(chrono.ChVector3d(1.8, 0, -0.25), chrono.QUNIT),
    chrono.ChColor(0.35, 0.35, 0.35),
)


# === Main loop: render and step the spring chain ===
try:
    while vis.Run() and sys.GetChTime() < SIM_END:

        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        for _ in range(STEPS_PER_FRAME):
            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= SIM_END:
                break

except (RuntimeError, ValueError) as exc:  # solver failure / invalid dynamic state
    print(f"Simulation failed: {exc}")
    raise
except (OSError, IOError) as exc:  # window or output path failure
    print(f"Output failed: {exc}")
    raise
finally:
    pass
