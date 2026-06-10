"""Epicyclic gear mechanism using a Chrono NSC multi-body system.

The model contains a fixed truss, a rotating carrier bar, a central sun gear,
and an orbiting planet gear. A single prescribed-speed rotational motor drives
the sun gear while an epicyclic gear constraint enforces the gear mesh, producing
planet spin and carrier motion in the Irrlicht visualization.
"""

import math

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr


# === Constants === named sizes and rates keep the mechanism easy to verify
TIME_STEP = 1.0e-3
SIM_END = 8.0
RENDER_FPS = 30.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

TRUSS_SIZE = chrono.ChVector3d(0.25, 0.10, 0.25)
BAR_LENGTH = 2.50
BAR_THICKNESS = 0.08
GEAR_THICKNESS = 0.18
SUN_RADIUS = 0.55
PLANET_RADIUS = 0.30
GEAR_CENTER_DISTANCE = SUN_RADIUS + PLANET_RADIUS
SUN_SPEED = 1.5
TOOTH_COUNT_SUN = 20
TOOTH_COUNT_PLANET = 12

SHAFT_FRAME = chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-math.pi / 2.0))
# === Helpers === repeated visual construction is kept local and deterministic
def color_shape(shape, red, green, blue):
    shape.SetColor(chrono.ChColor(red, green, blue))
    return shape


def add_gear_teeth(body, radius, tooth_count, tooth_size, color):
    for i in range(tooth_count):
        angle = 2.0 * math.pi * i / tooth_count
        tooth = chrono.ChVisualShapeBox(tooth_size, GEAR_THICKNESS * 1.08, tooth_size * 0.45)
        tooth.SetColor(color)
        offset = chrono.ChVector3d(
            (radius + 0.5 * tooth_size) * math.cos(angle),
            0.0,
            (radius + 0.5 * tooth_size) * math.sin(angle),
        )
        body.AddVisualShape(tooth, chrono.ChFramed(offset, chrono.QuatFromAngleY(-angle)))


# === System === pure jointed MBS, no contact or collision shapes
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0.0, -9.81, 0.0))


# === Bodies === fixed truss, carrier bar, sun gear, and planet gear
truss = chrono.ChBody()
truss.SetFixed(True)
truss.SetPos(chrono.ChVector3d(0.0, 0.0, 0.0))
truss_shape = color_shape(chrono.ChVisualShapeBox(TRUSS_SIZE), 0.35, 0.35, 0.38)
truss.AddVisualShape(truss_shape)
sys.AddBody(truss)

bar = chrono.ChBody()
bar.SetMass(1.0)
bar.SetInertiaXX(chrono.ChVector3d(0.08, 0.30, 0.08))
bar.SetPos(chrono.ChVector3d(GEAR_CENTER_DISTANCE / 2.0, 0.0, 0.0))
bar_shape = color_shape(chrono.ChVisualShapeBox(BAR_LENGTH, BAR_THICKNESS, BAR_THICKNESS), 0.10, 0.35, 0.85)
bar.AddVisualShape(bar_shape)
sys.AddBody(bar)

sun = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, SUN_RADIUS, GEAR_THICKNESS, 1000.0)
sun.SetPos(chrono.ChVector3d(0.0, 0.0, 0.0))
sun.GetVisualShape(0).SetColor(chrono.ChColor(0.95, 0.45, 0.12))
add_gear_teeth(sun, SUN_RADIUS, TOOTH_COUNT_SUN, 0.08, chrono.ChColor(0.95, 0.45, 0.12))
sys.AddBody(sun)

planet = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, PLANET_RADIUS, GEAR_THICKNESS, 1000.0)
planet.SetPos(chrono.ChVector3d(GEAR_CENTER_DISTANCE, 0.0, 0.0))
planet.GetVisualShape(0).SetColor(chrono.ChColor(0.20, 0.75, 0.35))
add_gear_teeth(planet, PLANET_RADIUS, TOOTH_COUNT_PLANET, 0.06, chrono.ChColor(0.20, 0.75, 0.35))
sys.AddBody(planet)

carrier_marker = chrono.ChBodyEasySphere(0.06, 1000.0, True, False)
carrier_marker.SetPos(chrono.ChVector3d(GEAR_CENTER_DISTANCE, 0.0, 0.0))
carrier_marker.GetVisualShape(0).SetColor(chrono.ChColor(0.05, 0.05, 0.05))
sys.AddBody(carrier_marker)


# === Joints === motorized sun gear, carrier hinge, planet pin, and epicyclic gear mesh
sun_motor = chrono.ChLinkMotorRotationSpeed()
sun_motor.Initialize(sun, truss, chrono.ChFramed(chrono.ChVector3d(0.0, 0.0, 0.0), chrono.QuatFromAngleX(math.pi / 2.0)))
sun_motor.SetSpeedFunction(chrono.ChFunctionConst(SUN_SPEED))
sys.AddLink(sun_motor)

bar_hinge = chrono.ChLinkLockRevolute()
bar_hinge.Initialize(bar, truss, chrono.ChFramed(chrono.ChVector3d(0.0, 0.0, 0.0), chrono.QuatFromAngleX(math.pi / 2.0)))
sys.AddLink(bar_hinge)

planet_pin = chrono.ChLinkLockRevolute()
planet_pin.Initialize(planet, bar, chrono.ChFramed(chrono.ChVector3d(GEAR_CENTER_DISTANCE, 0.0, 0.0), chrono.QuatFromAngleX(math.pi / 2.0)))
sys.AddLink(planet_pin)

marker_pin = chrono.ChLinkLockLock()
marker_pin.Initialize(carrier_marker, bar, chrono.ChFramed(chrono.ChVector3d(GEAR_CENTER_DISTANCE, 0.0, 0.0), chrono.QUNIT))
sys.AddLink(marker_pin)

gear_mesh = chrono.ChLinkLockGear()
gear_mesh.Initialize(sun, planet, chrono.ChFramed())
gear_mesh.SetFrameShaft1(SHAFT_FRAME)
gear_mesh.SetFrameShaft2(SHAFT_FRAME)
gear_mesh.SetTransmissionRatio(SUN_RADIUS / PLANET_RADIUS)
gear_mesh.SetEpicyclic(True)
gear_mesh.SetEnforcePhase(True)
sys.AddLink(gear_mesh)


# === Visualization === Irrlicht window and lighting built unconditionally
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Epicyclic gear train")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(2.0, 2.2, 3.2), chrono.ChVector3d(0.45, 0.0, 0.0))
vis.AddTypicalLights()
vis.AddGrid(
    0.25,
    0.25,
    16,
    16,
    chrono.ChCoordsysd(chrono.ChVector3d(0.0, -0.14, 0.0), chrono.QuatFromAngleX(math.pi / 2.0)),
    chrono.ChColor(0.45, 0.45, 0.45),
)


# === Main loop === render cadence and bounded time stepping


try:

    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        for _ in range(RENDER_EVERY):
            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:  # guard: Chrono solver or invalid state failure
    print(f"Simulation failed: {exc}")
    raise
except (OSError, IOError) as exc:  # guard: output directory or file-write failure
    print(f"Output failed: {exc}")
    raise
finally:
    pass
