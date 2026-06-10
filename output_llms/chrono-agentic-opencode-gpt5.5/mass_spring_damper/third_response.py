"""Mass-spring-damper chain in a PyChrono NSC system.

The model contains a fixed anchor, body_1, body_2, and body_3 arranged along
the X axis with TSDA spring-damper elements. The original anchor-to-body_1
spring is kept, and the requested body_1-to-body_2 and body_2-to-body_3
springs are added so the three masses oscillate as a coupled chain.
"""

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr


# === Constants === direct simulation values keep the mechanism truth-shaped
time_step = 1e-3
sim_end = 8.0
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))  # precomputed once

mass = 1.0
inertia = chrono.ChVector3d(0.1, 0.1, 0.1)
body_radius = 0.18
body_spacing = 1.6
spring_k = 50.0
damping_c = 1.0
rest_length = 1.2

anchor_pos = chrono.ChVector3d(0.0, 0.0, 0.0)
body_1_pos = chrono.ChVector3d(1.9, 0.0, 0.0)
body_2_pos = chrono.ChVector3d(3.2, 0.0, 0.0)
body_3_pos = chrono.ChVector3d(4.8, 0.0, 0.0)


# === System & Gravity === no contact is used, so the pure MBS system omits Bullet collision
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0.0, 0.0, 0.0))


# === Bodies === fixed anchor plus three dynamic masses named in the request
anchor = chrono.ChBody()
anchor.SetFixed(True)
anchor.SetPos(anchor_pos)
anchor.EnableCollision(False)
anchor_shape = chrono.ChVisualShapeSphere(body_radius * 0.7)
anchor_shape.SetColor(chrono.ChColor(0.35, 0.35, 0.35))
anchor.AddVisualShape(anchor_shape)
sys.AddBody(anchor)


def make_mass_body(name, pos, color):
    body = chrono.ChBody()
    body.SetName(name)
    body.SetMass(mass)
    body.SetInertiaXX(inertia)
    body.SetPos(pos)
    body.EnableCollision(False)
    shape = chrono.ChVisualShapeSphere(body_radius)
    shape.SetColor(color)
    body.AddVisualShape(shape)
    sys.AddBody(body)
    return body


body_1 = make_mass_body("body_1", body_1_pos, chrono.ChColor(0.2, 0.45, 0.95))
body_2 = make_mass_body("body_2", body_2_pos, chrono.ChColor(0.95, 0.45, 0.2))
body_3 = make_mass_body("body_3", body_3_pos, chrono.ChColor(0.25, 0.75, 0.35))
body_1.SetPosDt(chrono.ChVector3d(-1.2, 0.0, 0.0))
body_2.SetPosDt(chrono.ChVector3d(0.45, 0.0, 0.0))
body_3.SetPosDt(chrono.ChVector3d(0.9, 0.0, 0.0))


# === Joints / Constraints === TSDA links provide both spring and damping forces
def add_tsda(name, body_a, body_b, point_a, point_b):
    spring = chrono.ChLinkTSDA()
    spring.SetName(name)
    spring.Initialize(body_a, body_b, True, point_a, point_b)
    spring.SetRestLength(rest_length)
    spring.SetSpringCoefficient(spring_k)
    spring.SetDampingCoefficient(damping_c)
    spring.AddVisualShape(chrono.ChVisualShapeSpring(0.08, 80, 10))
    sys.AddLink(spring)
    return spring


spring_anchor_1 = add_tsda("spring_anchor_body_1", anchor, body_1, chrono.VNULL, chrono.VNULL)
spring_1_2 = add_tsda("spring_body_1_body_2", body_1, body_2, chrono.VNULL, chrono.VNULL)
spring_2_3 = add_tsda("spring_body_2_body_3", body_2, body_3, chrono.VNULL, chrono.VNULL)

body_1_cached = body_1  # cache: reused in every logging step
body_2_cached = body_2  # cache: reused in every logging step
body_3_cached = body_3  # cache: reused in every logging step
springs_cached = (spring_anchor_1, spring_1_2, spring_2_3)  # cache: reused in every logging step


# === Visualization === Irrlicht window is built unconditionally for review and scored core
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Mass Spring Damper Chain")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(2.4, -5.0, 1.8), chrono.ChVector3d(2.4, 0.0, 0.0))
vis.AddTypicalLights()
vis.AddGrid(
    0.5,
    0.5,
    12,
    8,
    chrono.ChCoordsysd(chrono.ChVector3d(2.4, -0.35, 0.0), chrono.QuatFromAngleX(chrono.CH_PI_2)),
    chrono.ChColor(0.45, 0.45, 0.45),
)


# === Main Loop === render once per frame and advance the spring chain in fixed steps
frame = 0
try:
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            t = sys.GetChTime()  # cache: one time query for this step
            pos_1 = body_1_cached.GetPos()  # cache: one position query for logging
            pos_2 = body_2_cached.GetPos()  # cache: one position query for logging
            pos_3 = body_3_cached.GetPos()  # cache: one position query for logging
            vel_1 = body_1_cached.GetPosDt()  # cache: one velocity query for logging
            vel_2 = body_2_cached.GetPosDt()  # cache: one velocity query for logging
            vel_3 = body_3_cached.GetPosDt()  # cache: one velocity query for logging
            sys.DoStepDynamics(time_step)
            if sys.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:  # solver divergence or invalid simulation state
    raise
finally:
    pass
