"""Motor-driven slider-crank MBS with spherical rod joints and a planar piston guide.

This PyChrono NSC simulation builds a fixed floor, a rotating crank, a connecting
rod, and a piston. The crank is driven by a prescribed-speed motor, the two
rod-end connections are ball-and-socket joints, and the piston is constrained by
a planar joint so it moves and rotates in the XY plane.
"""

import math
import traceback

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr


# === Constants ===
TIME_STEP = 1.0e-3
SIM_END = 4.0
CRANK_RADIUS = 1.0
ROD_LENGTH = 4.0
CRANK_OMEGA = math.pi
BODY_RADIUS = 0.08
BODY_DENSITY = 1000.0
PISTON_SIZE = chrono.ChVector3d(0.8, 0.6, 0.6)
PISTON_DENSITY = 1000.0
FLOOR_SIZE = chrono.ChVector3d(8.0, 0.2, 3.0)

ground_pos = chrono.ChVector3d(0.0, -0.25, 0.0)  # precomputed once
crank_angle0 = math.radians(35.0)  # precomputed once
crank_pin0 = chrono.ChVector3d(
    CRANK_RADIUS * math.cos(crank_angle0),
    CRANK_RADIUS * math.sin(crank_angle0),
    0.0,
)  # precomputed once
piston_x0 = crank_pin0.x + math.sqrt(ROD_LENGTH * ROD_LENGTH - crank_pin0.y * crank_pin0.y)  # precomputed once
piston_pos0 = chrono.ChVector3d(piston_x0, 0.0, 0.0)  # precomputed once
rod_mid0 = chrono.ChVector3d((crank_pin0.x + piston_pos0.x) * 0.5, crank_pin0.y * 0.5, 0.0)  # precomputed once
rod_angle0 = math.atan2(piston_pos0.y - crank_pin0.y, piston_pos0.x - crank_pin0.x)  # precomputed once


# === System & gravity ===
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0.0, -9.81, 0.0))


# === Bodies ===
floor = chrono.ChBodyEasyBox(FLOOR_SIZE.x, FLOOR_SIZE.y, FLOOR_SIZE.z, 1000.0, True, False)
floor.SetFixed(True)
floor.SetPos(ground_pos)
sys.Add(floor)

crank = chrono.ChBody()
crank.SetMass(1.0)
crank.SetInertiaXX(chrono.ChVector3d(0.2, 0.2, 0.2))
crank.SetPos(chrono.ChVector3d(crank_pin0.x * 0.5, crank_pin0.y * 0.5, 0.0))
crank.SetRot(chrono.QuatFromAngleZ(crank_angle0))
crank.EnableCollision(False)
crank_shape = chrono.ChVisualShapeCylinder(BODY_RADIUS, CRANK_RADIUS)
crank.AddVisualShape(crank_shape, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))
sys.AddBody(crank)

rod = chrono.ChBody()
rod.SetMass(1.0)
rod.SetInertiaXX(chrono.ChVector3d(0.2, 0.2, 0.2))
rod.SetPos(rod_mid0)
rod.SetRot(chrono.QuatFromAngleZ(rod_angle0))
rod.EnableCollision(False)
rod_shape = chrono.ChVisualShapeCylinder(BODY_RADIUS * 0.75, ROD_LENGTH)
rod.AddVisualShape(rod_shape, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))
sys.AddBody(rod)

piston = chrono.ChBodyEasyBox(
    PISTON_SIZE.x,
    PISTON_SIZE.y,
    PISTON_SIZE.z,
    PISTON_DENSITY,
    True,
    False,
)
piston.SetPos(piston_pos0)
sys.Add(piston)


# === Joints / constraints ===
crank_motor = chrono.ChLinkMotorRotationSpeed()
crank_motor.Initialize(crank, floor, chrono.ChFramed(chrono.VNULL, chrono.QUNIT))
crank_motor.SetSpeedFunction(chrono.ChFunctionConst(CRANK_OMEGA))
sys.AddLink(crank_motor)

crank_pin = chrono.ChLinkLockSpherical()
crank_pin.Initialize(
    crank,
    rod,
    True,
    chrono.ChFramed(chrono.ChVector3d(CRANK_RADIUS * 0.5, 0.0, 0.0), chrono.QUNIT),
    chrono.ChFramed(chrono.ChVector3d(-ROD_LENGTH * 0.5, 0.0, 0.0), chrono.QUNIT),
)
sys.AddLink(crank_pin)

wrist_pin = chrono.ChLinkLockSpherical()
wrist_pin.Initialize(
    rod,
    piston,
    True,
    chrono.ChFramed(chrono.ChVector3d(ROD_LENGTH * 0.5, 0.0, 0.0), chrono.QUNIT),
    chrono.ChFramed(chrono.VNULL, chrono.QUNIT),
)
sys.AddLink(wrist_pin)

piston_plane = chrono.ChLinkLockPlanar()
piston_plane.Initialize(piston, floor, chrono.ChFramed(piston_pos0, chrono.QUNIT))
sys.AddLink(piston_plane)


# === Visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Slider-crank with spherical rod joints and planar piston guide")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(3.5, -6.0, 4.0), chrono.ChVector3d(1.6, 0.0, 0.0))
vis.AddTypicalLights()
vis.AddGrid(0.5, 0.5, 16, 8, chrono.ChCoordsysd(), chrono.ChColor(0.35, 0.35, 0.35))


# === Main loop ===
try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        sys.DoStepDynamics(TIME_STEP)
except (RuntimeError, ValueError, OSError, IOError) as exc:
    traceback.print_exc()
    raise
finally:
    pass


# === Post-processing ===
