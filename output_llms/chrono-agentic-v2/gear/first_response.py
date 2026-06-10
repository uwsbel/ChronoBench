"""
Epicyclic (Planetary) Gear System — PyChrono 9.0.0, Irrlicht visualization.

System type : ChSystemNSC (rigid-body, no contact — pure jointed MBS)
Bodies      : fixed truss, rotating carrier bar, sun gear (driver), planet gear
Constraints : ChLinkMotorRotationSpeed (motor on sun), ChLinkLockRevolute (carrier
              pivot + planet pivot on carrier), ChLinkLockGear (epicyclic gear mesh)
Expected    : sun gear spins at constant rate driving the planet gear in the
              opposite sense (via the gear constraint) while the carrier bar
              rotates freely, reproducing a one-DOF epicyclic transmission.
"""

# === Imports ===
import math
import os
import csv
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# === Named constants ===
TIME_STEP     = 1e-3          # physics timestep [s]
SIM_END       = 10.0          # simulation duration [s]
RENDER_FPS    = 50.0          # rendering rate [fps]
render_every  = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

# Geometry
TRUSS_W, TRUSS_H, TRUSS_D = 0.5, 0.04, 0.15   # truss bar extents [m]
BAR_LEN    = 0.6     # carrier bar total length [m]
BAR_R      = 0.03    # carrier bar radius [m]
SUN_R      = 0.10    # sun gear pitch radius [m]
PLANET_R   = 0.05    # planet gear pitch radius [m]
GEAR_H     = 0.04    # gear disc thickness [m]
GEAR_DENS  = 2000.0  # gear density [kg/m³]
MOTOR_SPD  = math.pi # motor angular speed [rad/s] (π rad/s)

# Carrier bar pivot = origin; planet sits at sun_R + planet_R from origin
SUN_POS    = chrono.ChVector3d(0.0, 0.0, 0.0)
PLANET_POS = chrono.ChVector3d(SUN_R + PLANET_R, 0.0, 0.0)

# Review-only recording setup

# === System & gravity (Y-up, NSC, pure jointed MBS — no collision system needed) ===
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

# === Bodies ===

# --- Fixed truss (serves as the ground reference for motor and carrier pivot) ---
truss = chrono.ChBody()
truss.SetMass(1.0)
truss.SetInertiaXX(chrono.ChVector3d(0.1, 0.1, 0.1))
truss.SetPos(chrono.ChVector3d(0.0, 0.0, 0.0))
truss.SetFixed(True)
truss_vs = chrono.ChVisualShapeBox(TRUSS_W, TRUSS_H, TRUSS_D)
truss_vs.SetColor(chrono.ChColor(0.6, 0.5, 0.3))
truss.AddVisualShape(truss_vs)
sys.AddBody(truss)

# --- Carrier bar (rotates about origin, carries the planet gear) ---
bar = chrono.ChBody()
bar.SetMass(0.5)
bar.SetInertiaXX(chrono.ChVector3d(0.01, 0.05, 0.05))
bar.SetPos(chrono.ChVector3d(BAR_LEN * 0.5 - 0.0, 0.0, 0.05))  # offset in Z so visible
bar_cyl = chrono.ChVisualShapeCylinder(BAR_R, BAR_LEN)
bar.AddVisualShape(bar_cyl, chrono.ChFramed(
    chrono.ChVector3d(BAR_LEN * 0.5 - 0.0 - (BAR_LEN * 0.5 - 0.0), 0.0, 0.0),
    chrono.QuatFromAngleY(chrono.CH_PI_2)))
bar_cyl.SetColor(chrono.ChColor(0.3, 0.6, 0.8))
sys.AddBody(bar)

# --- Sun gear (disc spinning at motor speed) ---
sun = chrono.ChBodyEasyCylinder(chrono.ChAxis_Z, SUN_R, GEAR_H, GEAR_DENS)
sun.SetPos(SUN_POS)
sun.GetVisualShape(0).SetColor(chrono.ChColor(0.9, 0.6, 0.1))
sys.Add(sun)

# --- Planet gear (meshes with sun, pivots on carrier bar) ---
planet = chrono.ChBodyEasyCylinder(chrono.ChAxis_Z, PLANET_R, GEAR_H, GEAR_DENS)
planet.SetPos(PLANET_POS)
planet.GetVisualShape(0).SetColor(chrono.ChColor(0.2, 0.8, 0.3))
sys.Add(planet)

# === Joints / constraints ===

# 1. Motor: sun gear spins at constant speed relative to fixed truss (full motor-link)
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(sun, truss,
                 chrono.ChFramed(SUN_POS, chrono.QUNIT))
motor.SetSpeedFunction(chrono.ChFunctionConst(MOTOR_SPD))
sys.AddLink(motor)

# 2. Carrier bar pivot: bar revolves about origin relative to truss
carrier_pivot = chrono.ChLinkLockRevolute()
carrier_pivot.Initialize(bar, truss,
                          chrono.ChFramed(chrono.ChVector3d(0.0, 0.0, 0.05),
                                          chrono.QUNIT))
sys.AddLink(carrier_pivot)

# 3. Planet pivot: planet revolves about its own center on the carrier bar
#    bar is positioned so bar local origin coincides with planet center in Z-layer
planet_pivot = chrono.ChLinkLockRevolute()
planet_pivot.Initialize(planet, bar,
                         chrono.ChFramed(PLANET_POS + chrono.ChVector3d(0, 0, 0.05),
                                         chrono.QUNIT))
sys.AddLink(planet_pivot)

# 4. Epicyclic gear constraint between sun and planet
gear_link = chrono.ChLinkLockGear()
gear_link.Initialize(sun, planet, chrono.ChFramed())
gear_link.SetFrameShaft1(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-math.pi / 2)))
gear_link.SetFrameShaft2(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-math.pi / 2)))
gear_link.SetTransmissionRatio(SUN_R / PLANET_R)
gear_link.SetEnforcePhase(True)
gear_link.SetEpicyclic(False)   # external mesh (teeth on outer surfaces)
sys.AddLink(gear_link)

# === Visualization (Irrlicht — Initialize FIRST, then scene elements AFTER) ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Epicyclic Gear System")
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.8, 0.6, 0.8), chrono.ChVector3d(0.2, 0.0, 0.0))
vis.AddTypicalLights()
vis.AddGrid(0.1, 0.1, 20, 20,
            chrono.ChCoordsysd(chrono.ChVector3d(0, -0.05, 0), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))

# === Main loop ===
frame = 0


try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
