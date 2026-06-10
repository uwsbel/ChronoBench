"""Multi-mass spring-damper chain (PyChrono 9.0.1, Irrlicht).

Models a horizontal chain of three sliding masses anchored to a fixed wall:

    wall (fixed) --spring/damper--> body_1 --spring/damper--> body_2 --spring/damper--> body_3

System type: NSC (ChSystemNSC), pure jointed multi-body — no collision/contact.
Each mass slides along a fixed X guide (prismatic joint to ground) and is coupled
to its neighbour by a ChLinkTSDA translational spring-damper carrying a visible
ChVisualShapeSpring. The masses are released displaced from their rest spacing,
so the chain oscillates and the damping bleeds energy until the system settles to
its static equilibrium. Expected behavior: decaying longitudinal oscillation of
all three bodies, body_1 leading and body_3 trailing.
"""

import os
import math

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# === Constants === geometry / physics parameters; positions derived from these
time_step = 1e-3
sim_end = 10.0
render_fps = 50.0

gravity_z = -9.81

# masses of the three sliding bodies
mass_1 = 1.0
mass_2 = 1.0
mass_3 = 1.0

# body half-extents (full box extents are 2x these); used for inertia + visuals
body_hx = 0.15
body_hy = 0.15
body_hz = 0.15

# spring-damper parameters (identical springs between successive bodies)
spring_k = 50.0
damping_c = 2.0
rest_length = 1.0

# spring visual appearance
coil_radius = 0.06
coil_resolution = 80
coil_turns = 12

# wall (fixed anchor) position and rest spacing of the chain along +X
wall_x = 0.0
support_z = 0.0

# Rest layout: each body sits one rest_length downstream of the previous anchor.
rest_x1 = wall_x + rest_length
rest_x2 = rest_x1 + rest_length
rest_x3 = rest_x2 + rest_length

# Initial displacement: compress the chain so it oscillates after release.
init_offset = 0.4
init_x1 = rest_x1 - init_offset
init_x2 = rest_x2 - init_offset
init_x3 = rest_x3 - init_offset

# guide / spring axis is world +X (chain extends along X)
guide_axis_quat = chrono.Q_ROTATE_Z_TO_X  # maps frame local +Z onto world +X

render_every = max(1, round(1.0 / (render_fps * time_step)))  # precomputed once

# === System & gravity === single NSC system; no contact in this mechanism
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, gravity_z))

# === Solver === PSOR + warm start: stiff spring chain needs a stable iterative solve
sys.SetSolverType(chrono.ChSolver.Type_PSOR)
solver = sys.GetSolver().AsIterative()
solver.SetMaxIterations(100)
solver.SetTolerance(1e-10)
solver.EnableWarmStart(True)  # reuse previous step's solution -> spring convergence


def make_inertia(m, hx, hy, hz):
    """Solid-box principal inertia for a body of mass m and half-extents h*."""
    ix = (1.0 / 12.0) * m * ((2 * hy) ** 2 + (2 * hz) ** 2)
    iy = (1.0 / 12.0) * m * ((2 * hx) ** 2 + (2 * hz) ** 2)
    iz = (1.0 / 12.0) * m * ((2 * hx) ** 2 + (2 * hy) ** 2)
    return chrono.ChVector3d(ix, iy, iz)


def make_mass(m, x):
    """Create one sliding box body (no collision shape) at world (x, 0, support_z)."""
    body = chrono.ChBody()
    body.SetMass(m)
    body.SetInertiaXX(make_inertia(m, body_hx, body_hy, body_hz))
    body.SetPos(chrono.ChVector3d(x, 0, support_z))
    box = chrono.ChVisualShapeBox(2 * body_hx, 2 * body_hy, 2 * body_hz)
    box.SetColor(chrono.ChColor(0.2, 0.5, 0.9))
    body.AddVisualShape(box)
    sys.AddBody(body)
    return body


# === Bodies === fixed wall anchor + three sliding masses along +X
wall = chrono.ChBody()
wall.SetFixed(True)
wall.SetPos(chrono.ChVector3d(wall_x, 0, support_z))
wall_box = chrono.ChVisualShapeBox(0.1, 0.6, 0.6)
wall_box.SetColor(chrono.ChColor(0.5, 0.5, 0.5))
wall.AddVisualShape(wall_box)
sys.AddBody(wall)

body_1 = make_mass(mass_1, init_x1)
body_2 = make_mass(mass_2, init_x2)
body_3 = make_mass(mass_3, init_x3)

# === Joints / constraints === prismatic X-guides keep every mass on the chain axis
guide_quat = guide_axis_quat  # cache: same guide orientation reused for all guides
for body in (body_1, body_2, body_3):
    guide = chrono.ChLinkLockPrismatic()
    guide.Initialize(body, wall, chrono.ChFramed(body.GetPos(), guide_quat))
    sys.AddLink(guide)


def make_spring(b_anchor, b_mass):
    """Translational spring-damper from b_anchor to b_mass with a visible coil."""
    spring = chrono.ChLinkTSDA()
    spring.Initialize(b_anchor, b_mass, False,
                      b_anchor.GetPos(), b_mass.GetPos())
    spring.SetRestLength(rest_length)
    spring.SetSpringCoefficient(spring_k)
    spring.SetDampingCoefficient(damping_c)
    spring.AddVisualShape(chrono.ChVisualShapeSpring(coil_radius, coil_resolution, coil_turns))
    sys.AddLink(spring)
    return spring

# === Springs === one spring-damper per successive pair (wall->1, 1->2, 2->3)
spring_w1 = make_spring(wall, body_1)
spring_12 = make_spring(body_1, body_2)
spring_23 = make_spring(body_2, body_3)

# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Multi-mass spring-damper chain")
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(rest_x2, -5.0, 2.0), chrono.ChVector3d(rest_x2, 0, support_z))
vis.AddTypicalLights()
vis.AddGrid(0.5, 0.5, 40, 40,
            chrono.ChCoordsysd(chrono.ChVector3d(rest_x2, 0, support_z - body_hz), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))

# === Main loop === real-time render-cadence loop; physics advanced in batches


# cache: body handles fetched once, reused every step
b1, b2, b3 = body_1, body_2, body_3

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
except (RuntimeError, ValueError) as exc:   # solver divergence / invalid state
    import traceback
    traceback.print_exc()
    raise

# === Post-processing === assemble review video + table, then drop raw frames
