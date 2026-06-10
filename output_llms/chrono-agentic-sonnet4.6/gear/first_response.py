"""
Epicyclic gear system simulation using PyChrono.

Models a system of epicyclic (planetary) gears consisting of:
- A fixed truss (ground body)
- A rotating carrier bar driven by a gear motor at constant speed
- Two gears: a sun gear (driven by motor against truss) and a planet gear
  (meshing with the sun via ChLinkLockGear epicyclic constraint)
- The carrier bar revolves around the sun axis while the planet orbits

System type: NSC (Non-Smooth Contact, pure jointed MBS — no contact/collision).
Expected behavior: the motor enforces a constant angular velocity on the sun gear
about the central axis; the planet gear rotates and orbits on the carrier bar;
the bar sweeps around the central axis at the motor speed.
"""

import math
import os
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# === Constants ===
TIME_STEP = 1e-3          # physics step size [s]
SIM_END   = 10.0          # simulation duration [s]
RENDER_FPS = 50.0         # target render rate [Hz]

# Gear geometry
SUN_RADIUS    = 0.5       # sun gear radius [m]
PLANET_RADIUS = 0.25      # planet gear radius [m]
CARRIER_LEN   = SUN_RADIUS + PLANET_RADIUS  # bar half-length from center [m]

MOTOR_SPEED   = chrono.CH_PI   # 0.5 rev/s for the sun gear [rad/s]

# Body parameters (simple direct assignments — truth style)
TRUSS_MASS  = 1.0
TRUSS_INERT = chrono.ChVector3d(1.0, 1.0, 1.0)

BAR_MASS    = 1.0
BAR_INERT   = chrono.ChVector3d(0.05, 1.0, 1.0)

SUN_MASS    = 1.0
SUN_INERT   = chrono.ChVector3d(0.5, 0.5, 0.5)

PLANET_MASS = 0.5
PLANET_INERT = chrono.ChVector3d(0.1, 0.1, 0.1)

# === System & gravity ===
# Pure jointed MBS — no contact; omit SetCollisionSystemType per codegen rules
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

# === Bodies ===

# Fixed truss — the anchor for the motor and for the bar pivot
truss = chrono.ChBody()
truss.SetMass(TRUSS_MASS)
truss.SetInertiaXX(TRUSS_INERT)
truss.SetPos(chrono.ChVector3d(0, 0, 0))
truss.SetFixed(True)
sys.AddBody(truss)

truss_vis = chrono.ChVisualShapeBox(0.1, 0.1, 0.1)
truss_vis.SetColor(chrono.ChColor(0.4, 0.4, 0.4))
truss.AddVisualShape(truss_vis)

# Carrier bar — rotates about Z axis centered at origin
bar = chrono.ChBody()
bar.SetMass(BAR_MASS)
bar.SetInertiaXX(BAR_INERT)
bar.SetPos(chrono.ChVector3d(CARRIER_LEN / 2.0, 0, 0))
sys.AddBody(bar)

bar_vis = chrono.ChVisualShapeCylinder(0.05, CARRIER_LEN)
bar_vis.SetColor(chrono.ChColor(0.6, 0.4, 0.2))
# Cylinder default axis is body-local Z; rotate to body-local X for a horizontal bar
bar.AddVisualShape(bar_vis, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))

# Sun gear — centered at origin, driven by motor against truss
sun_gear = chrono.ChBody()
sun_gear.SetMass(SUN_MASS)
sun_gear.SetInertiaXX(SUN_INERT)
sun_gear.SetPos(chrono.ChVector3d(0, 0, 0))
sys.AddBody(sun_gear)

sun_vis = chrono.ChVisualShapeCylinder(SUN_RADIUS, 0.1)
sun_vis.SetColor(chrono.ChColor(0.8, 0.2, 0.2))
sun_gear.AddVisualShape(sun_vis)  # cylinder default Z coincides with rotation axis Z

# Planet gear — rides on the carrier bar
planet_center = chrono.ChVector3d(SUN_RADIUS + PLANET_RADIUS, 0, 0)
planet_gear = chrono.ChBody()
planet_gear.SetMass(PLANET_MASS)
planet_gear.SetInertiaXX(PLANET_INERT)
planet_gear.SetPos(planet_center)
sys.AddBody(planet_gear)

planet_vis = chrono.ChVisualShapeCylinder(PLANET_RADIUS, 0.1)
planet_vis.SetColor(chrono.ChColor(0.2, 0.5, 0.9))
planet_gear.AddVisualShape(planet_vis)  # cylinder default Z coincides with rotation axis Z

# === Joints / constraints ===

# Motor: drives the sun gear (and implicitly the carrier) against the fixed truss.
# ChLinkMotorRotationSpeed is a FULL motor-link — no separate revolute needed.
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(
    sun_gear, truss,
    chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT)  # pivot at origin, Z-axis
)
motor.SetSpeedFunction(chrono.ChFunctionConst(MOTOR_SPEED))
sys.AddLink(motor)

# Revolute: bar pivots about Z at the origin, constrained to the truss.
# The bar carries the planet, so it must also be linked to the truss at the hub.
bar_pivot = chrono.ChLinkLockRevolute()
bar_pivot.Initialize(
    bar, truss,
    chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT)  # Z-axis revolute
)
sys.AddLink(bar_pivot)

# Revolute: planet spins freely on the bar end
planet_on_bar = chrono.ChLinkLockRevolute()
planet_on_bar.Initialize(
    planet_gear, bar,
    chrono.ChFramed(planet_center, chrono.QUNIT)  # Z-axis revolute at planet center
)
sys.AddLink(planet_on_bar)

# Epicyclic gear constraint between sun and planet
# SetEpicyclic(True) = internal/ring mesh (planet orbits inside; here it orbits outside
# the sun, so we use the standard external pair without epicyclic flag)
gear_link = chrono.ChLinkLockGear()
gear_link.Initialize(planet_gear, sun_gear, chrono.ChFramed())
gear_link.SetFrameShaft1(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-math.pi / 2)))
gear_link.SetFrameShaft2(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-math.pi / 2)))
gear_link.SetTransmissionRatio(SUN_RADIUS / PLANET_RADIUS)
gear_link.SetEnforcePhase(True)
sys.AddLink(gear_link)

# === Visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Epicyclic Gear System")
vis.Initialize()                                                # Initialize FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, -3, 2), chrono.ChVector3d(0, 0, 0))  # AFTER Initialize
vis.AddTypicalLights()

# === Main loop ===
render_every = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once


frame = 0
try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            t = sys.GetChTime()                                # cache: reused each inner step
            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / bad parameter
    import traceback; traceback.print_exc()
    raise
finally:
    pass  # writers closed in review-only block below
