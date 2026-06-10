"""Slider-crank mechanism (PyChrono 9.0.1, ChSystemNSC, Irrlicht).

Models a motor-driven planar slider-crank linkage built from four rigid bodies:
a fixed floor/ground, a rotating crank, a connecting rod, and a sliding piston.
The crank turns about a fixed ground pivot driven by a rotation-speed motor; the
crank-rod and rod-piston pins are ball-and-socket (spherical) joints, and the
piston is constrained to the world x-y plane by a plane-plane (planar) joint so
it may translate in x and y and spin about the plane normal (world Z).

System type: NSC (non-smooth). This is a PURE jointed multi-body mechanism with
no contact between bodies, so no collision system is configured. Gravity acts
along -Y; the linkage lies in the x-y plane. Expected behavior: the crank spins
at constant angular speed, the rod swings, and the piston oscillates back and
forth along x with a small in-plane wobble permitted by the spherical/planar
topology.
"""

# === Imports ===
import os
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# === Parameters === geometry / physics constants (no bare literals downstream)
time_step = 1e-3          # integration step [s]
sim_end = 6.0             # simulation duration [s]
render_fps = 50.0         # review render cadence [frames/s]

crank_radius = 0.6        # crank throw: pivot to crank pin [m]
crank_thick = 0.1         # crank/rod cross-section [m]
rod_length = 2.0          # connecting-rod length (pin to pin) [m]
piston_size = 0.4         # piston cube full edge [m]
crank_omega = 2.0 * math.pi   # crank angular speed [rad/s] -> 1 rev/s

crank_density = 2700.0    # aluminium-like density [kg/m^3]
rod_density = 2700.0
piston_density = 2700.0

# --- Derived initial geometry (crank at angle 0: pin on +X axis) ---
pivot_pos = chrono.ChVector3d(0, 0, 0)                 # ground pivot at origin
crank_pin0 = chrono.ChVector3d(crank_radius, 0, 0)     # crank pin world pos at t=0
# Piston starts on the x-axis; rod spans from crank pin to piston pin.
piston_x0 = crank_radius + rod_length                  # precomputed once
piston_pos0 = chrono.ChVector3d(piston_x0, 0, 0)
rod_mid0 = chrono.ChVector3d((crank_pin0.x + piston_pos0.x) * 0.5, 0, 0)

# === System & gravity === one NSC system; gravity along -Y (linkage in x-y plane)
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))
# Pure jointed MBS (no contact between bodies) -> no collision system configured.

# === Bodies === floor (fixed), crank, connecting rod, piston
# Fixed floor / ground reference (also the anchor for ground joints).
floor = chrono.ChBody()
floor.SetFixed(True)
floor.SetName("floor")
floor_vis = chrono.ChVisualShapeBox(8.0, 0.2, 4.0)     # full extents
floor_vis.SetColor(chrono.ChColor(0.4, 0.45, 0.5))
floor.AddVisualShape(floor_vis, chrono.ChFramed(chrono.ChVector3d(2.0, -1.2, 0), chrono.QUNIT))
sys.AddBody(floor)

# Crank: thin box rotating about the ground pivot; origin at its geometric center.
crank = chrono.ChBody()
crank.SetName("crank")
crank.SetMass(crank_radius * crank_thick * crank_thick * crank_density)
crank.SetInertiaXX(chrono.ChVector3d(1e-3, 1e-3, 1e-2))
crank.SetPos(chrono.ChVector3d(crank_radius * 0.5, 0, 0))   # midpoint pivot->pin
crank_vis = chrono.ChVisualShapeBox(crank_radius, crank_thick, crank_thick)
crank_vis.SetColor(chrono.ChColor(0.8, 0.3, 0.2))
crank.AddVisualShape(crank_vis)
sys.AddBody(crank)

# Connecting rod: thin box spanning crank pin to piston pin; origin at its center.
rod = chrono.ChBody()
rod.SetName("rod")
rod.SetMass(rod_length * crank_thick * crank_thick * rod_density)
rod.SetInertiaXX(chrono.ChVector3d(1e-3, 1e-1, 1e-1))
rod.SetPos(rod_mid0)
rod_vis = chrono.ChVisualShapeBox(rod_length, crank_thick, crank_thick)
rod_vis.SetColor(chrono.ChColor(0.2, 0.5, 0.8))
rod.AddVisualShape(rod_vis)
sys.AddBody(rod)

# Piston: solid cube constrained to the x-y plane; origin at its center.
piston = chrono.ChBody()
piston.SetName("piston")
piston_mass = piston_size ** 3 * piston_density
piston.SetMass(piston_mass)
piston_I = (1.0 / 6.0) * piston_mass * piston_size ** 2
piston.SetInertiaXX(chrono.ChVector3d(piston_I, piston_I, piston_I))
piston.SetPos(piston_pos0)
piston_vis = chrono.ChVisualShapeBox(piston_size, piston_size, piston_size)
piston_vis.SetColor(chrono.ChColor(0.3, 0.7, 0.3))
piston.AddVisualShape(piston_vis)
sys.AddBody(piston)

# === Joints / constraints === crank pivot + motor, spherical pins, planar piston
# Crank-to-ground revolute hinge about world Z (linkage spins in the x-y plane).
crank_pivot = chrono.ChLinkLockRevolute()
crank_pivot.Initialize(crank, floor, chrono.ChFramed(pivot_pos, chrono.QUNIT))
sys.AddLink(crank_pivot)

# Rotation-speed motor drives the crank at a constant angular rate about the pivot.
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(crank, floor, chrono.ChFramed(pivot_pos, chrono.QUNIT))
motor.SetSpeedFunction(chrono.ChFunctionConst(crank_omega))
sys.AddLink(motor)

# Crank-rod pin: SPHERICAL (ball-and-socket) at the crank pin (world crank_pin0).
# Body-local frames: crank pin at +crank_radius/2 along crank X; rod near end at -rod_length/2.
crank_rod = chrono.ChLinkLockSpherical()
crank_rod.Initialize(
    crank, rod, True,
    chrono.ChFramed(chrono.ChVector3d(crank_radius * 0.5, 0, 0), chrono.QUNIT),   # crank far end (pin)
    chrono.ChFramed(chrono.ChVector3d(-rod_length * 0.5, 0, 0), chrono.QUNIT),    # rod near end
)
sys.AddLink(crank_rod)

# Rod-piston pin: SPHERICAL (ball-and-socket) at the piston center (world piston_pos0).
rod_piston = chrono.ChLinkLockSpherical()
rod_piston.Initialize(
    rod, piston, True,
    chrono.ChFramed(chrono.ChVector3d(rod_length * 0.5, 0, 0), chrono.QUNIT),     # rod far end
    chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),                    # piston center
)
sys.AddLink(rod_piston)

# Piston-floor PLANAR (plane-plane) joint: the joint frame local +Z is the plane
# normal. With QUNIT the plane is the world x-y plane, so the piston may translate
# in x and y and rotate about world Z, while z-translation and x/y-rotation are locked.
piston_plane = chrono.ChLinkLockPlanar()
piston_plane.Initialize(piston, floor, chrono.ChFramed(piston_pos0, chrono.QUNIT))
sys.AddLink(piston_plane)

# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)   # gravity along -Y -> Y is up
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Slider-Crank Mechanism")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(1.4, 1.6, 5.0), chrono.ChVector3d(1.4, 0, 0))
vis.AddTypicalLights()
vis.AddGrid(0.5, 0.5, 40, 40,
            chrono.ChCoordsysd(chrono.ChVector3d(2.0, -1.1, 0),
                               chrono.QuatFromAngleX(chrono.CH_PI_2)),
            chrono.ChColor(0.4, 0.4, 0.4))          # ground reference grid in x-z

# === Main loop === real-time render-cadence loop; physics in inner batch
render_every = max(1, round(1.0 / (render_fps * time_step)))   # precomputed once

get_time = sys.GetChTime           # cache: getter fetched once, reused every step
piston_body = piston               # cache: body handle reused for logging



frame = 0
try:   # guard the time-stepping loop against solver divergence
    while vis.Run() and get_time() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            sys.DoStepDynamics(time_step)
            if get_time() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / invalid state
    import traceback
    traceback.print_exc()
    raise
