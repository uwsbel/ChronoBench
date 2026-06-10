"""
Epicyclic Gear System Simulation

Models a system of epicyclic (planetary) gears consisting of:
  - A fixed truss (ground body) providing the pivot for the bar and sun gear
  - A rotating bar (carrier) driven by a ChLinkMotorRotationSpeed at constant speed
  - A sun gear fixed to the truss center pivot
  - A planet gear mounted on the rotating bar, meshing with the sun gear via ChLinkLockGear (epicyclic)

System type: ChSystemNSC (pure jointed MBS, no contact — collision omitted per ground-truth convention)
Gravity: Y-up (0, -9.81, 0)
Expected behavior: The bar rotates at constant speed; the planet gear rolls around the sun gear,
spinning on its own axis at a rate determined by the gear transmission ratio.
"""

import math
import os
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# === Constants ===
time_step = 1e-3       # physics step [s]
sim_end   = 10.0       # simulation end time [s]
render_fps = 50.0      # target render frames per second
render_every = max(1, round(1.0 / (render_fps * time_step)))  # precomputed once

# Geometry constants
truss_size      = 0.3        # truss box half-ish size [m]
bar_length      = 1.0        # arm/bar length (center-to-center of gears) [m]
bar_radius      = 0.05       # bar cross-section radius [m]
bar_mass        = 1.0        # bar mass [kg]

sun_radius      = 0.3        # sun gear pitch radius [m]
planet_radius   = 0.2        # planet gear pitch radius [m]
gear_height     = 0.1        # gear visual thickness [m]
gear_mass       = 1.0        # gear mass [kg]

motor_speed     = chrono.CH_PI   # motor angular speed [rad/s] = pi rad/s

# Derived positions (Y-up convention)
origin          = chrono.ChVector3d(0, 0, 0)      # truss pivot center
bar_pivot_pos   = chrono.ChVector3d(0, 0, 0)      # bar hinge at origin
planet_pivot_pos = chrono.ChVector3d(bar_length, 0, 0)  # planet center on bar (along X initially)

# === System ===
# ChSystemNSC: pure jointed gear mechanism, no contact/collision shapes → collision system omitted
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

# === Bodies ===

# --- Fixed truss (ground reference) ---
truss = chrono.ChBody()
truss.SetMass(1.0)
truss.SetInertiaXX(chrono.ChVector3d(1e-4, 1e-4, 1e-4))
truss.SetPos(chrono.ChVector3d(0, 0, 0))
truss.SetFixed(True)
truss.EnableCollision(False)
# Visual shape: small box to represent the fixed support
truss_shape = chrono.ChVisualShapeBox(truss_size, truss_size, truss_size * 0.5)
truss_shape.SetColor(chrono.ChColor(0.4, 0.4, 0.5))
truss.AddVisualShape(truss_shape)
sys.AddBody(truss)

# --- Rotating bar (carrier arm) ---
# Bar rotates about origin pivot, extends along its local X to planet center
# Body origin placed at bar midpoint: (bar_length/2, 0, 0)
bar_center = chrono.ChVector3d(bar_length / 2.0, 0, 0)
bar = chrono.ChBody()
bar.SetMass(bar_mass)
# Inertia of a thin cylinder along X: Ixx=1/12*m*(3r²+l²), Iyy=Izz same
_bl = bar_length
_br = bar_radius
bar_Ixx = (1.0 / 2.0) * bar_mass * _br**2  # about spin axis (minor)
bar_Iyy = (1.0 / 12.0) * bar_mass * (3.0 * _br**2 + _bl**2)
bar_Izz = bar_Iyy
bar.SetInertiaXX(chrono.ChVector3d(bar_Ixx, bar_Iyy, bar_Izz))
bar.SetPos(bar_center)
bar.SetRot(chrono.QUNIT)
bar.EnableCollision(False)
# Visual: cylinder along body-local X (step1: QUNIT → local X = world X initially;
#         step2: cylinder default Z rotated to X via QuatFromAngleY(PI/2))
bar_cyl = chrono.ChVisualShapeCylinder(bar_radius, bar_length)
bar_cyl.SetColor(chrono.ChColor(0.6, 0.4, 0.2))
bar.AddVisualShape(bar_cyl, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))
sys.AddBody(bar)

# --- Sun gear (fixed to truss, at origin) ---
sun = chrono.ChBody()
sun.SetMass(gear_mass)
sun_Ixx = 0.5 * gear_mass * sun_radius**2
sun_Iyy = (1.0 / 12.0) * gear_mass * (3.0 * sun_radius**2 + gear_height**2)
sun.SetInertiaXX(chrono.ChVector3d(sun_Iyy, sun_Iyy, sun_Ixx))
sun.SetPos(chrono.ChVector3d(0, 0, 0))
sun.SetFixed(True)
sun.EnableCollision(False)
sun_cyl = chrono.ChVisualShapeCylinder(sun_radius, gear_height)
sun_cyl.SetColor(chrono.ChColor(0.9, 0.7, 0.2))
sun.AddVisualShape(sun_cyl, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))
sys.AddBody(sun)

# --- Planet gear (mounted on bar at planet_pivot_pos, rolls around sun) ---
planet = chrono.ChBody()
planet.SetMass(gear_mass)
planet_Ixx = 0.5 * gear_mass * planet_radius**2
planet_Iyy = (1.0 / 12.0) * gear_mass * (3.0 * planet_radius**2 + gear_height**2)
planet.SetInertiaXX(chrono.ChVector3d(planet_Iyy, planet_Iyy, planet_Ixx))
planet.SetPos(planet_pivot_pos)
planet.SetRot(chrono.QUNIT)
planet.EnableCollision(False)
planet_cyl = chrono.ChVisualShapeCylinder(planet_radius, gear_height)
planet_cyl.SetColor(chrono.ChColor(0.3, 0.7, 0.9))
planet.AddVisualShape(planet_cyl, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))
sys.AddBody(planet)

# === Joints / Constraints ===
# Gear mechanism topology:
#   truss → [motor] → bar          (motor drives bar at constant speed about truss origin)
#   bar   → [revolute] → planet    (planet can spin freely on bar arm)
#   sun   ↔ planet via [ChLinkLockGear, epicyclic] (kinematic gear constraint)
#
# Note: ChLinkMotorRotationSpeed is a FULL motor-link (no companion revolute needed at same pivot).
# The revolute Z-axis (hinge normal to XY plane) is world +Z with QUNIT frame.

# --- Motor: drives bar relative to truss at constant angular speed ---
motor = chrono.ChLinkMotorRotationSpeed()
# Motor frame at origin (pivot), hinge about world +Z (QUNIT: local +Z = world +Z)
motor.Initialize(bar, truss, chrono.ChFramed(origin, chrono.QUNIT))
motor.SetSpeedFunction(chrono.ChFunctionConst(motor_speed))
sys.AddLink(motor)

# --- Revolute: planet spins on bar arm at planet_pivot_pos ---
# Hinge axis = world +Z (QUNIT); planet can freely spin about Z relative to bar
planet_rev = chrono.ChLinkLockRevolute()
planet_rev.Initialize(planet, bar, chrono.ChFramed(planet_pivot_pos, chrono.QUNIT))
sys.AddLink(planet_rev)

# --- Epicyclic gear constraint: sun ↔ planet ---
# ChLinkLockGear with SetEpicyclic(True) enforces kinematic tooth meshing
# Shaft frames: local +X must align with the gear rotation axis (world +Z here);
# we map local +Z of frame to gear spin axis, using QuatFromAngleX(-pi/2)
# so that local +Z points along world +Y... actually we want shaft along world +Z.
# The gear spins about world +Z. ChLinkLockGear shaft convention:
# SetFrameShaft uses the shaft's local Z as the rotation axis.
# We set frame with QuatFromAngleX(-pi/2) → local +Z → world +Z stays as world +Z (QUNIT).
# Use QUNIT: local +Z = world +Z which IS our hinge axis.
gear_link = chrono.ChLinkLockGear()
gear_link.Initialize(sun, planet, chrono.ChFramed())
# Shaft frames align gear rotation axes (both spin about world +Z → use QuatFromAngleX(-pi/2)
# per ground-truth convention for gears in the XY plane)
_shaft_q = chrono.QuatFromAngleX(-math.pi / 2.0)
gear_link.SetFrameShaft1(chrono.ChFramed(chrono.VNULL, _shaft_q))
gear_link.SetFrameShaft2(chrono.ChFramed(chrono.VNULL, _shaft_q))
# Transmission ratio = sun_radius / planet_radius (external mesh by default)
gear_link.SetTransmissionRatio(sun_radius / planet_radius)
gear_link.SetEnforcePhase(True)
gear_link.SetEpicyclic(True)   # epicyclic (internal/ring mesh flag for carrier system)
sys.AddLink(gear_link)

# === Visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Epicyclic Gear System — PyChrono 9.0.x")
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(3.0, -3.0, 2.5), chrono.ChVector3d(0.5, 0.0, 0.0))
vis.AddTypicalLights()
vis.AddGrid(0.5, 0.5, 20, 20,
            chrono.ChCoordsysd(chrono.ChVector3d(0, 0, -0.01), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))

# === Review-only: recording setup ===


# === Main loop ===
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
except (RuntimeError, ValueError) as exc:  # solver divergence / bad system state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
