"""
Epicyclic Gear System Simulation
===============================
Plan type: mbs (pure multi-body mechanics)

A system of epicyclic (planetary) gears consisting of:
- Fixed truss (ground reference)
- Sun gear (center) driven by a rotational motor at constant speed
- Planet gear meshed with sun via ChLinkLockGear
- Rotating carrier bar connecting the planet to the central shaft
- Ring gear (fixed) meshing with the planet

The motor enforces a constant rotation speed on the sun gear,
causing the planet to orbit and the carrier to rotate.
"""

import csv
import math
import os

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr


# === Named constants ===
# Gear geometry
SUN_RADIUS = 0.15       # sun gear radius [m]
PLANET_RADIUS = 0.075   # planet gear radius [m]
RING_INNER_R = 0.30      # ring gear inner radius [m]
GEAR_THICKNESS = 0.03    # gear disc thickness [m]

# Motor speed
MOTOR_SPEED = 2.0        # rad/s constant rotation for sun gear

# Simulation
TIME_STEP = 1e-3        # high precision for mechanism
SIM_END = 10.0
RENDER_FPS = 50.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))

# Contact method: NSC for rigid MBS (no contact needed for pure jointed gear train)
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

# === Bodies ===

# -- Fixed truss --
truss = chrono.ChBody()
truss.SetFixed(True)
truss.SetMass(1.0)
truss.SetInertiaXX(chrono.ChVector3d(1e-6, 1e-6, 1e-6))
truss.SetPos(chrono.ChVector3d(0, 0, 0))
vis_truss = chrono.ChVisualShapeBox(0.8, 0.02, 0.8)
vis_truss.SetColor(chrono.ChColor(0.4, 0.4, 0.4))
truss.AddVisualShape(vis_truss, chrono.ChFramed(chrono.ChVector3d(0, -0.01, 0), chrono.QUNIT))
sys.AddBody(truss)

# -- Carrier bar --
# Rotates about Z at origin; bar extends from center to planet position
CARRIER_LEN = SUN_RADIUS + PLANET_RADIUS  # = 0.225
bar = chrono.ChBody()
bar.SetMass(1.0)
bar.SetInertiaXX(chrono.ChVector3d(0.05, 1.0, 1.0))
bar.SetPos(chrono.ChVector3d(CARRIER_LEN / 2.0, 0, 0))
sys.AddBody(bar)

bar_vis = chrono.ChVisualShapeCylinder(0.05, CARRIER_LEN)
bar_vis.SetColor(chrono.ChColor(0.6, 0.4, 0.2))
# Cylinder axis is body-local Z; rotate to body-local X for a horizontal bar along X
bar.AddVisualShape(bar_vis, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))

# -- Sun gear -- centered at origin, driven by motor
sun_gear = chrono.ChBody()
sun_gear.SetMass(1.0)
sun_gear.SetInertiaXX(chrono.ChVector3d(0.5, 0.5, 0.5))
sun_gear.SetPos(chrono.ChVector3d(0, 0, 0))
sys.AddBody(sun_gear)

sun_vis = chrono.ChVisualShapeCylinder(SUN_RADIUS, GEAR_THICKNESS)
sun_vis.SetColor(chrono.ChColor(0.8, 0.2, 0.2))
# Cylinder default axis is body-local Z = world Z (shaft direction)
sun_gear.AddVisualShape(sun_vis)

# -- Planet gear -- rides on carrier bar
PLANET_POS = chrono.ChVector3d(SUN_RADIUS + PLANET_RADIUS, 0, 0)
planet_gear = chrono.ChBody()
planet_gear.SetMass(0.5)
planet_gear.SetInertiaXX(chrono.ChVector3d(0.1, 0.1, 0.1))
planet_gear.SetPos(PLANET_POS)
sys.AddBody(planet_gear)

planet_vis = chrono.ChVisualShapeCylinder(PLANET_RADIUS, GEAR_THICKNESS)
planet_vis.SetColor(chrono.ChColor(0.2, 0.5, 0.9))
# Cylinder default Z = world Z
planet_gear.AddVisualShape(planet_vis)

# -- Ring gear (fixed) --
ring_body = chrono.ChBody()
ring_body.SetFixed(True)
ring_body.SetMass(1.0)
ring_body.SetInertiaXX(chrono.ChVector3d(1e-6, 1e-6, 1e-6))
ring_body.SetPos(chrono.ChVector3d(0, 0, 0))
sys.AddBody(ring_body)

ring_vis = chrono.ChVisualShapeCylinder(RING_INNER_R + PLANET_RADIUS, GEAR_THICKNESS * 1.5)
ring_vis.SetColor(chrono.ChColor(0.3, 0.3, 0.5))
ring_body.AddVisualShape(ring_vis, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))

# === Joints / constraints ===

# Motor: drives the sun gear against the fixed truss (full motor-link)
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(
    sun_gear, truss,
    chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT)  # pivot at origin, Z-axis
)
motor.SetSpeedFunction(chrono.ChFunctionConst(MOTOR_SPEED))
sys.AddLink(motor)

# Revolute: carrier bar pivots about Z at the origin, constrained to the truss
bar_pivot = chrono.ChLinkLockRevolute()
bar_pivot.Initialize(
    bar, truss,
    chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT)  # Z-axis revolute at origin
)
sys.AddLink(bar_pivot)

# Revolute: planet spins freely on the carrier bar end
planet_on_bar = chrono.ChLinkLockRevolute()
planet_on_bar.Initialize(
    planet_gear, bar,
    chrono.ChFramed(PLANET_POS, chrono.QUNIT)  # Z-axis revolute at planet center
)
sys.AddLink(planet_on_bar)

# Epicyclic gear constraint between planet and sun
# SetEpicyclic(True) = planet meshes with sun (external gear pair)
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
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.8, -0.8, 0.5), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()
vis.AddGrid(0.1, 0.1, 40, 40,
            chrono.ChCoordsysd(chrono.ChVector3d(0, 0, -0.01), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))

# === CSV logging ===
csv_file = open("simulation_data.csv", "w", newline="")
fieldnames = ["time", "bar_angle_deg", "sun_omega", "planet_omega"]
data_writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
data_writer.writeheader()


render_every = RENDER_EVERY

frame = 0
try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            t = sys.GetChTime()
            bar_rot = bar.GetRot()
            bar_angle = bar_rot.GetCardanAnglesZYX().x * 180.0 / math.pi
            sun_w = sun_gear.GetAngVelParent().z
            planet_w = planet_gear.GetAngVelParent().z
            data_writer.writerow({
                "time": round(t, 6),
                "bar_angle_deg": round(bar_angle, 4),
                "sun_omega": round(sun_w, 4),
                "planet_omega": round(planet_w, 4),
            })
            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= SIM_END:
                break
finally:
    csv_file.close()
