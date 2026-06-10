"""Crank-slider mechanism driven by a constant-speed rotational motor.

Models a classic crank-slider (reciprocating engine) linkage with PyChrono's
non-smooth-contact system (ChSystemNSC), although the mechanism is a PURE jointed
multibody system with NO contact/collision: every relative motion is enforced by
ideal joints, so no collision system is configured.

Bodies:
  - truss   : fixed ground/frame holding the crank pivot and the slider guide.
  - crank   : crankshaft rotating about a fixed pivot, spun by the motor.
  - rod     : connecting rod linking the crank pin to the piston.
  - piston  : slider translating along the world X guide.

Topology (planar XY motion, gravity along -Y, all hinge axes along world +Z):
  - revolute  truss-crank  at the crank pivot (+ constant-speed motor).
  - revolute  crank-rod    at the crank pin.
  - revolute  rod-piston   at the wrist pin.
  - prismatic piston-truss  along the world X guide.

Expected behavior: the motor spins the crank at a constant angular speed, the rod
converts that rotation into a reciprocating (back-and-forth) translation of the
piston along X — the piston X position oscillates sinusoidally and never leaves
the guide axis (Y and Z of the piston stay constant).
"""

import os
import math

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# === Parameters === geometry / physics constants (no bare position literals downstream)
time_step = 1.0e-3          # integration step [s]
sim_end = 8.0               # simulation duration [s]
render_fps = 50.0           # review-frame cadence [frames/s]

crank_radius = 0.5          # crank pin offset from pivot [m]
rod_length = 1.5            # connecting-rod length (pin-to-pin) [m]
motor_speed = 2.0 * math.pi # crank angular speed [rad/s] -> 1 rev/s

crank_mass = 2.0            # crankshaft mass [kg]
rod_mass = 1.0              # connecting-rod mass [kg]
piston_mass = 1.5           # piston mass [kg]

pivot_pos = chrono.ChVector3d(0.0, 0.0, 0.0)            # fixed crank pivot (world origin)
crank_pin0 = chrono.ChVector3d(crank_radius, 0.0, 0.0)  # crank pin at angle 0 (along +X)
# Wrist-pin sits on the X guide; from the crank pin the rod reaches to piston_x0 along X.
piston_x0 = crank_pin0.x + math.sqrt(rod_length * rod_length - crank_pin0.y * crank_pin0.y)
wrist_pin0 = chrono.ChVector3d(piston_x0, 0.0, 0.0)     # rod-piston pin at angle 0
rod_center0 = (crank_pin0 + wrist_pin0) * 0.5           # rod COM midway between its pins

render_every = max(1, round(1.0 / (render_fps * time_step)))  # physics steps per frame; precomputed once

# === System & gravity === single NSC system, gravity along -Y (planar XY mechanism)
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0.0, -9.81, 0.0))
# Pure jointed linkage with no contact -> no SetCollisionSystemType (truss/crank/rod/piston
# never collide; their motion is fully constrained by the revolute/prismatic joints).

# === Bodies === truss (floor), crankshaft, connecting rod, piston
# Truss: fixed frame carrying the pivot and the slider guide (visualized as a slab).
truss = chrono.ChBody()
truss.SetFixed(True)
truss.SetPos(chrono.ChVector3d(0.0, 0.0, 0.0))
sys.AddBody(truss)
truss_slab = chrono.ChVisualShapeBox(4.0, 0.2, 1.0)     # full extents [m]
truss_slab.SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
truss.AddVisualShape(truss_slab, chrono.ChFramed(chrono.ChVector3d(piston_x0 * 0.5, -0.6, 0.0), chrono.QUNIT))
# Pivot pin stub on the truss (structural, visual-only).
truss_pin = chrono.ChVisualShapeCylinder(0.06, 0.4)
truss_pin.SetColor(chrono.ChColor(0.3, 0.3, 0.3))
truss.AddVisualShape(truss_pin, chrono.ChFramed(pivot_pos, chrono.QUNIT))  # cylinder axis = local Z

# Crankshaft: rotates about the pivot; body origin at its COM (midway pivot->pin).
crank = chrono.ChBody()
crank.SetMass(crank_mass)
crank.SetInertiaXX(chrono.ChVector3d(0.02, 0.02, 0.02))
crank.SetPos((pivot_pos + crank_pin0) * 0.5)
sys.AddBody(crank)
crank_arm = chrono.ChVisualShapeBox(crank_radius, 0.12, 0.08)  # arm along local X
crank_arm.SetColor(chrono.ChColor(0.8, 0.2, 0.2))
crank.AddVisualShape(crank_arm)

# Connecting rod: links crank pin to wrist pin; body-local X along the rod direction.
rod = chrono.ChBody()
rod.SetMass(rod_mass)
rod.SetInertiaXX(chrono.ChVector3d(0.01, 0.2, 0.2))
rod.SetPos(rod_center0)
sys.AddBody(rod)
rod_cyl = chrono.ChVisualShapeCylinder(0.05, rod_length)  # default axis = local Z
rod_cyl.SetColor(chrono.ChColor(0.2, 0.4, 0.8))
rod.AddVisualShape(rod_cyl, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))  # Z->X

# Piston: slides along the world X guide; origin at the wrist pin.
piston = chrono.ChBody()
piston.SetMass(piston_mass)
piston.SetInertiaXX(chrono.ChVector3d(0.02, 0.02, 0.02))
piston.SetPos(wrist_pin0)
sys.AddBody(piston)
piston_cyl = chrono.ChVisualShapeCylinder(0.18, 0.4)      # default axis = local Z
piston_cyl.SetColor(chrono.ChColor(0.2, 0.7, 0.3))
piston.AddVisualShape(piston_cyl, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))  # Z->X

# === Joints / constraints === revolutes (hinge about +Z) + motor + slider prismatic
# Truss-crank revolute at the fixed pivot (hinge axis world +Z -> QUNIT in XY plane).
rev_pivot = chrono.ChLinkLockRevolute()
rev_pivot.Initialize(crank, truss, chrono.ChFramed(pivot_pos, chrono.QUNIT))
sys.AddLink(rev_pivot)

# Constant-speed motor on the same pivot spins the crank at motor_speed.
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(crank, truss, chrono.ChFramed(pivot_pos, chrono.QUNIT))
motor.SetSpeedFunction(chrono.ChFunctionConst(motor_speed))
sys.AddLink(motor)

# Crank-rod revolute at the crank pin (body-local frames so each marker is on its body).
rev_crank_rod = chrono.ChLinkLockRevolute()
rev_crank_rod.Initialize(
    crank, rod, True,
    chrono.ChFramed(chrono.ChVector3d(crank_radius * 0.5, 0.0, 0.0), chrono.QUNIT),   # crank far end (local +X)
    chrono.ChFramed(chrono.ChVector3d(-rod_length * 0.5, 0.0, 0.0), chrono.QUNIT),    # rod near end (local -X)
)
sys.AddLink(rev_crank_rod)

# Rod-piston revolute at the wrist pin (fixed-guide linkage keeps this a REVOLUTE, not prismatic).
rev_rod_piston = chrono.ChLinkLockRevolute()
rev_rod_piston.Initialize(
    rod, piston, True,
    chrono.ChFramed(chrono.ChVector3d(rod_length * 0.5, 0.0, 0.0), chrono.QUNIT),     # rod far end (local +X)
    chrono.ChFramed(chrono.VNULL, chrono.QUNIT),                                      # piston origin
)
sys.AddLink(rev_rod_piston)

# Piston-truss prismatic along the world X guide (frame local +Z -> world +X via Q_ROTATE_Z_TO_X).
prism_piston = chrono.ChLinkLockPrismatic()
prism_piston.Initialize(piston, truss, chrono.ChFramed(wrist_pin0, chrono.Q_ROTATE_Z_TO_X))
sys.AddLink(prism_piston)

# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)         # gravity along -Y -> Y is up
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Crank-Slider Mechanism")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(piston_x0 * 0.5, 1.5, 5.0), chrono.ChVector3d(piston_x0 * 0.5, 0.0, 0.0))
vis.AddTypicalLights()
vis.AddGrid(0.5, 0.5, 40, 40,
            chrono.ChCoordsysd(chrono.ChVector3d(piston_x0 * 0.5, -0.7, 0.0),
                               chrono.QuatFromAngleX(chrono.CH_PI_2)),
            chrono.ChColor(0.4, 0.4, 0.4))                # ground reference grid in the XZ plane

# === Main loop === drive the crank, log piston/crank state, render at fixed cadence


try:
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            sys.DoStepDynamics(time_step)
            if sys.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / invalid mechanism state
    import traceback
    traceback.print_exc()
    raise

# === Post-processing === flush the CSV, assemble the review video + plot, drop raw frames
