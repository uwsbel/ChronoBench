"""Epicyclic gear train in a PyChrono NSC multi-body system.

The model contains a fixed truss, a carrier bar rotating about the truss
center, a speed-driven sun gear, and a planet gear pinned to the carrier.
The gears are connected by an epicyclic gear constraint so the motor-driven
sun gear enforces steady planetary motion. Irrlicht renders the mechanism
with colored materials and lighting.
"""

import math
import traceback

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr


# === Constants ===
# Geometry and timing are named so the gear layout is easy to verify.
time_step = 0.001
sim_end = 2.0

sun_radius = 0.35
planet_radius = 0.22
gear_width = 0.10
carrier_radius = sun_radius + planet_radius
carrier_length = carrier_radius
bar_radius = 0.035
shaft_radius = 0.055
shaft_height = 0.22
gear_density = 7800.0
bar_mass = 4.0
motor_speed = 0.75

center = chrono.ChVector3d(0, 0, 0)
planet_center = chrono.ChVector3d(carrier_radius, 0, 0)
carrier_midpoint = chrono.ChVector3d(0.5 * carrier_length, 0, 0)
hinge_frame = chrono.ChFramed(center, chrono.QUNIT)
planet_frame = chrono.ChFramed(planet_center, chrono.QUNIT)
shaft_frame = chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-math.pi / 2.0))


# === System & Gravity ===
# A jointed gear train has no contact, so the NSC system uses no collision system.
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))


def color_shape(shape, r, g, b):
    """Apply a simple diffuse color to a Chrono visual shape."""
    shape.SetColor(chrono.ChColor(r, g, b))


def make_cylinder(name, radius, height, density, position, color):
    """Create one visual cylinder with a cached body reference for constraints."""
    body = chrono.ChBodyEasyCylinder(chrono.ChAxis_Z, radius, height, density)
    body.SetName(name)
    body.SetPos(position)
    color_shape(body.GetVisualShape(0), *color)
    body.EnableCollision(False)
    sys.AddBody(body)
    return body


def add_face_marker(body, radius, color):
    """Add a colored spoke on a gear face so rotation is visible in video."""
    spoke = chrono.ChVisualShapeBox(radius, 0.035, 0.025)
    color_shape(spoke, *color)
    body.AddVisualShape(spoke, chrono.ChFramed(chrono.ChVector3d(0.5 * radius, 0, 0.07), chrono.QUNIT))
    dot = chrono.ChVisualShapeSphere(0.045)
    color_shape(dot, *color)
    body.AddVisualShape(dot, chrono.ChFramed(chrono.ChVector3d(0.8 * radius, 0, 0.09), chrono.QUNIT))


# === Bodies ===
# The fixed truss is the visual and kinematic reference for all central joints.
truss = chrono.ChBody()
truss.SetName("fixed truss")
truss.SetFixed(True)
truss.SetPos(center)
truss.EnableCollision(False)
truss.SetMass(1.0)
truss.SetInertiaXX(chrono.ChVector3d(1, 1, 1))
post_shape = chrono.ChVisualShapeCylinder(shaft_radius, shaft_height)
color_shape(post_shape, 0.35, 0.35, 0.40)
truss.AddVisualShape(post_shape, chrono.ChFramed(chrono.VNULL, chrono.QUNIT))
base_shape = chrono.ChVisualShapeBox(1.55, 0.06, 0.05)
color_shape(base_shape, 0.28, 0.28, 0.32)
truss.AddVisualShape(base_shape, chrono.ChFramed(chrono.ChVector3d(0.2, -0.52, -0.08), chrono.QUNIT))
sys.AddBody(truss)

carrier = chrono.ChBody()
carrier.SetName("rotating carrier bar")
carrier.SetMass(bar_mass)
carrier.SetInertiaXX(chrono.ChVector3d(0.20, 0.20, 0.20))
carrier.SetPos(carrier_midpoint)
carrier.SetRot(chrono.QUNIT)
carrier.EnableCollision(False)
bar_shape = chrono.ChVisualShapeCylinder(bar_radius, carrier_length)
color_shape(bar_shape, 0.95, 0.65, 0.15)
carrier.AddVisualShape(bar_shape, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))
hub_shape = chrono.ChVisualShapeSphere(0.08)
color_shape(hub_shape, 0.95, 0.65, 0.15)
carrier.AddVisualShape(hub_shape, chrono.ChFramed(chrono.ChVector3d(-0.5 * carrier_length, 0, 0), chrono.QUNIT))
end_shape = chrono.ChVisualShapeSphere(0.08)
color_shape(end_shape, 0.95, 0.65, 0.15)
carrier.AddVisualShape(end_shape, chrono.ChFramed(chrono.ChVector3d(0.5 * carrier_length, 0, 0), chrono.QUNIT))
sys.AddBody(carrier)

sun = make_cylinder("motorized sun gear", sun_radius, gear_width, gear_density, center, (0.15, 0.45, 0.95))
planet = make_cylinder("planet gear", planet_radius, gear_width, gear_density, planet_center, (0.90, 0.25, 0.20))
add_face_marker(sun, sun_radius, (1.0, 1.0, 0.05))
add_face_marker(planet, planet_radius, (0.05, 1.0, 1.0))

sun_body = sun  # cache: fetched once for motor and logging
planet_body = planet  # cache: fetched once for joints and logging
carrier_body = carrier  # cache: fetched once for hinge and logging


# === Joints / Constraints ===
# The motor is a full rotational link; no duplicate revolute is added at the sun.
sun_motor = chrono.ChLinkMotorRotationSpeed()
sun_motor.Initialize(sun_body, truss, hinge_frame)
sun_motor.SetSpeedFunction(chrono.ChFunctionConst(motor_speed))
sys.AddLink(sun_motor)

carrier_hinge = chrono.ChLinkLockRevolute()
carrier_hinge.Initialize(carrier_body, truss, hinge_frame)
sys.AddLink(carrier_hinge)

planet_hinge = chrono.ChLinkLockRevolute()
planet_hinge.Initialize(planet_body, carrier_body, planet_frame)
sys.AddLink(planet_hinge)

gear_constraint = chrono.ChLinkLockGear()
gear_constraint.Initialize(sun_body, planet_body, chrono.ChFramed())
gear_constraint.SetFrameShaft1(shaft_frame)
gear_constraint.SetFrameShaft2(shaft_frame)
gear_constraint.SetTransmissionRatio(sun_radius / planet_radius)
gear_constraint.SetEnforcePhase(True)
gear_constraint.SetEpicyclic(True)
sys.AddLink(gear_constraint)


# === Visualization ===
# Irrlicht is initialized before adding all scene elements.
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Epicyclic Gear Train")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(1.4, -2.0, 1.2), chrono.ChVector3d(0.25, 0, 0))
vis.AddTypicalLights()
vis.AddLight(chrono.ChVector3d(1.5, -2.5, 3.0), 8.0, chrono.ChColor(0.95, 0.95, 0.90))
vis.AddGrid(
    0.2,
    0.2,
    12,
    12,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, -0.12), chrono.QUNIT),
    chrono.ChColor(0.35, 0.35, 0.35),
)


# === Main Loop ===
# The scored loop is a real-time Irrlicht render plus one dynamics step.

try:
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        sys.DoStepDynamics(time_step)
except (RuntimeError, ValueError, OSError) as exc:
    traceback.print_exc()
    raise
