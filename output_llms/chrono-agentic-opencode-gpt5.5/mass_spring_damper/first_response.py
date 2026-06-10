"""Single mass-spring-damper system in an NSC PyChrono model.

A fixed ground marker and one translating rigid mass are connected by a
ChLinkTSDA with linear spring and damping coefficients. Gravity is disabled so
the motion highlights the damped horizontal oscillation of the mass, while the
Irrlicht scene shows the mass, anchor, spring, lighting, camera, and grid.
"""

import traceback

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr


# === Parameters === named constants keep the mechanism dimensions and timing explicit
time_step = 1e-3
sim_end = 6.0
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))  # precomputed once

mass_value = 1.0
mass_size = 0.22
mass_inertia = 0.01
anchor_pos = chrono.ChVector3d(0.0, 0.0, 0.0)
mass_initial_pos = chrono.ChVector3d(1.5, 0.0, 0.0)
rest_length = 1.0
spring_k = 50.0
damping_c = 1.0
coil_radius = 0.08


# === System & gravity === pure jointed MBS with no collision/contact
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0.0, 0.0, 0.0))


# === Bodies === fixed ground anchor and one dynamic mass with visible geometry
ground = chrono.ChBody()
ground.SetFixed(True)
ground.SetPos(anchor_pos)
ground_marker = chrono.ChVisualShapeSphere(0.08)
ground_marker.SetColor(chrono.ChColor(0.2, 0.2, 0.2))
ground.AddVisualShape(ground_marker)
sys.AddBody(ground)

mass = chrono.ChBody()
mass.SetMass(mass_value)
mass.SetInertiaXX(chrono.ChVector3d(mass_inertia, mass_inertia, mass_inertia))
mass.SetPos(mass_initial_pos)
mass_shape = chrono.ChVisualShapeBox(mass_size, mass_size, mass_size)
mass_shape.SetColor(chrono.ChColor(0.1, 0.35, 0.9))
mass.AddVisualShape(mass_shape)
sys.AddBody(mass)


# === Joints / constraints === ChLinkTSDA provides the linear spring-damper force
spring = chrono.ChLinkTSDA()
spring.Initialize(ground, mass, True, chrono.ChVector3d(0.0, 0.0, 0.0), chrono.ChVector3d(0.0, 0.0, 0.0))
spring.SetRestLength(rest_length)
spring.SetSpringCoefficient(spring_k)
spring.SetDampingCoefficient(damping_c)
spring.AddVisualShape(chrono.ChVisualShapeSpring(coil_radius, 80, 12))
sys.AddLink(spring)


# === Visualization === Irrlicht window initialized before adding scene elements
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Mass-Spring-Damper with ChLinkTSDA")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(2.8, 1.6, 2.0), chrono.ChVector3d(0.7, 0.0, 0.0))
vis.AddTypicalLights()
vis.AddGrid(
    0.25,
    0.25,
    16,
    8,
    chrono.ChCoordsysd(chrono.ChVector3d(0.8, -0.12, 0.0), chrono.QuatFromAngleX(chrono.CH_PI_2)),
    chrono.ChColor(0.45, 0.45, 0.45),
)


# === Main loop === render at video cadence and advance physics in fixed steps
mass_body = mass  # cache: protagonist body reused for logging every step
spring_link = spring  # cache: spring state queried repeatedly in the loop
try:

    frame = 0
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            sim_time = sys.GetChTime()  # cache: time is used for logging and stop checks
            mass_pos = mass_body.GetPos()  # cache: position queried once per step
            mass_vel = mass_body.GetPosDt()  # cache: velocity queried once per step
            sys.DoStepDynamics(time_step)
            if sys.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:  # solver divergence / invalid state guard
    traceback.print_exc()
    raise
except (OSError, IOError) as exc:  # output directory or CSV file creation guard
    traceback.print_exc()
    raise
finally:
    pass
