"""Crank-slider mechanism in PyChrono (rigid multi-body, NSC system).

Models a classic slider-crank: a fixed floor/truss, a crankshaft spun at a
constant angular speed by a rotational-speed motor, a connecting rod, and a
piston sliding on a fixed guide. The mechanism is planar in the world XZ plane
with gravity along -Z; all hinge axes are the world +Y direction.

Topology (each joint links two DISTINCT bodies):
  crank  <-> truss   : ChLinkMotorRotationSpeed (full motor-link, no revolute)
  crank  <-> rod     : ChLinkLockRevolute (crank pin)
  rod    <-> piston  : ChLinkLockRevolute (wrist pin)
  piston <-> truss   : ChLinkLockPrismatic (slides along world +X)

Expected behavior: the motor spins the crank at constant speed; the rod converts
that rotation into reciprocating linear motion of the piston along the X guide.
No contact/collision is present (pure jointed MBS), so no collision system is set.
"""

import os
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# === Constants === geometry/physics; positions derived from these (no bare literals)
time_step = 1.0e-3            # integration step [s]
sim_end = 6.0                # total simulated time [s]
render_fps = 50.0            # review-video frame rate

crank_radius = 1.0           # crank throw: pin distance from crank axis [m]
rod_length = 4.0             # connecting-rod length [m]
crank_speed = math.pi        # prescribed crank angular speed [rad/s]

# Hinge axis for every revolute is world +Y (motion plane is XZ).
q_hinge_y = chrono.QuatFromAngleAxis(chrono.CH_PI_2, chrono.VECT_X)  # precomputed once

# Derived initial world positions (crank angle = 0 -> pin on +X axis).
crank_axis_pos = chrono.ChVector3d(0, 0, 0)
crank_pin_pos = chrono.ChVector3d(crank_radius, 0, 0)
piston_x = crank_radius + rod_length          # crank/rod collinear at start
piston_pos = chrono.ChVector3d(piston_x, 0, 0)
rod_center = chrono.ChVector3d((crank_pin_pos.x + piston_x) * 0.5, 0, 0)

# === System & gravity === single NSC system; gravity along -Z (XZ motion plane)
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))

# === Bodies === truss (fixed), crank, connecting rod, piston (all visualized)
# Floor / truss: fixed reference frame for the whole mechanism.
truss = chrono.ChBody()
truss.SetFixed(True)
truss.SetPos(crank_axis_pos)
truss_vis = chrono.ChVisualShapeBox(0.6, 1.4, 0.6)
truss_vis.SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
truss.AddVisualShape(truss_vis, chrono.ChFramed(chrono.ChVector3d(-0.6, 0, 0), chrono.QUNIT))
sys.AddBody(truss)

# Crankshaft: short disc spinning about world Y at the truss axis.
crank = chrono.ChBody()
crank.SetMass(1.0)
crank.SetInertiaXX(chrono.ChVector3d(0.2, 0.2, 0.2))
crank.SetPos(crank_axis_pos)
crank_disc = chrono.ChVisualShapeCylinder(0.18, 0.4)
crank.AddVisualShape(crank_disc, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(chrono.CH_PI_2)))
crank_arm = chrono.ChVisualShapeBox(crank_radius, 0.3, 0.16)
crank.AddVisualShape(crank_arm, chrono.ChFramed(chrono.ChVector3d(crank_radius * 0.5, 0, 0), chrono.QUNIT))
sys.AddBody(crank)

# Connecting rod: thin bar between crank pin and piston, body origin at its center.
rod = chrono.ChBody()
rod.SetMass(1.0)
rod.SetInertiaXX(chrono.ChVector3d(0.1, 1.0, 1.0))
rod.SetPos(rod_center)
rod_vis = chrono.ChVisualShapeBox(rod_length, 0.2, 0.12)
rod_vis.SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
rod.AddVisualShape(rod_vis)
sys.AddBody(rod)

# Piston: block that slides on the fixed X guide.
piston = chrono.ChBody()
piston.SetMass(1.0)
piston.SetInertiaXX(chrono.ChVector3d(0.5, 0.5, 0.5))
piston.SetPos(piston_pos)
piston_vis = chrono.ChVisualShapeBox(0.7, 0.7, 0.7)
piston_vis.SetTexture(chrono.GetChronoDataFile("textures/pinkwhite.png"))
piston.AddVisualShape(piston_vis)
sys.AddBody(piston)

# === Joints / constraints === motor + crank-pin + wrist-pin revolutes + guide prismatic
# Crank <-> truss: prescribed-speed motor (FULL motor-link; no companion revolute).
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(crank, truss, chrono.ChFramed(crank_axis_pos, q_hinge_y))
motor.SetSpeedFunction(chrono.ChFunctionConst(crank_speed))
sys.AddLink(motor)

# Crank <-> rod: revolute at the crank pin (hinge about world Y).
crank_rod = chrono.ChLinkLockRevolute()
crank_rod.Initialize(crank, rod, chrono.ChFramed(crank_pin_pos, q_hinge_y))
sys.AddLink(crank_rod)

# Rod <-> piston: revolute at the wrist pin (hinge about world Y).
rod_piston = chrono.ChLinkLockRevolute()
rod_piston.Initialize(rod, piston, chrono.ChFramed(piston_pos, q_hinge_y))
sys.AddLink(rod_piston)

# Piston <-> truss: prismatic sliding along world +X (frame local +Z -> world X).
piston_guide = chrono.ChLinkLockPrismatic()
piston_guide.Initialize(piston, truss, chrono.ChFramed(piston_pos, chrono.Q_ROTATE_Z_TO_X))
sys.AddLink(piston_guide)

# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Crank-Slider Mechanism")
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(2.5, -6.0, 2.5), chrono.ChVector3d(2.5, 0, 0))
vis.AddTypicalLights()
vis.AddGrid(0.5, 0.5, 40, 40,
            chrono.ChCoordsysd(chrono.ChVector3d(0, 0, -1.0), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))

# === Main loop === render-cadence outer loop; physics in inner batch of render_every
render_every = max(1, round(1.0 / (render_fps * time_step)))   # precomputed once
piston_handle = piston                                              # cache: reused every step

os.makedirs("cam", exist_ok=True)   # guard against missing output dir
try:

    frame = 0
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            sys.DoStepDynamics(time_step)
            if sys.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / invalid state
    import traceback; traceback.print_exc()
    raise
