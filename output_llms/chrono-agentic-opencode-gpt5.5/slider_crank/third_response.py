"""Motor-driven PyChrono slider-crank mechanism with altered constraints.

The model uses a ChSystemNSC pure multi-body assembly with no contact: a fixed
floor/truss, a speed-driven crank, a connecting rod, and a piston. The crank-pin
and wrist-pin connections are spherical ball-and-socket joints, and the piston is
constrained to the XY motion plane by a planar joint to the fixed floor. The
expected behavior is a crank-driven linkage that keeps the piston in the XY
plane while allowing in-plane translation and rotation.
"""

import math

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr


# === Constants === direct mechanism parameters and precomputed run cadence
time_step = 1e-3
sim_end = 6.0
render_fps = 30.0
render_every = max(1, round(1.0 / (render_fps * time_step)))  # precomputed once

crank_radius = 0.40
rod_length = 1.20
crank_angle0 = math.radians(35.0)
crank_speed = math.pi
crank_pin0 = chrono.ChVector3d(
    crank_radius * math.cos(crank_angle0),
    crank_radius * math.sin(crank_angle0),
    0.0,
)
piston_x0 = crank_pin0.x + math.sqrt(rod_length * rod_length - crank_pin0.y * crank_pin0.y)
piston_pos0 = chrono.ChVector3d(piston_x0, 0.0, 0.0)
rod_center0 = chrono.ChVector3d((crank_pin0.x + piston_pos0.x) * 0.5, crank_pin0.y * 0.5, 0.0)
rod_angle0 = math.atan2(piston_pos0.y - crank_pin0.y, piston_pos0.x - crank_pin0.x)

# === System & gravity === pure jointed MBS, so no collision system is required
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0.0, -9.81, 0.0))


# === Bodies === fixed truss, rotating crank, connecting rod, and planar piston
floor = chrono.ChBody()
floor.SetName("floor")
floor.SetFixed(True)
sys.AddBody(floor)

floor_shape = chrono.ChVisualShapeBox(2.4, 0.05, 0.08)
floor_shape.SetColor(chrono.ChColor(0.35, 0.35, 0.35))
floor.AddVisualShape(floor_shape, chrono.ChFramed(chrono.ChVector3d(0.8, -0.35, 0.0), chrono.QUNIT))

crank = chrono.ChBody()
crank.SetName("crank")
crank.SetMass(1.0)
crank.SetInertiaXX(chrono.ChVector3d(0.02, 0.02, 0.02))
crank.SetPos(chrono.ChVector3d(0.0, 0.0, 0.0))
crank.SetRot(chrono.QuatFromAngleZ(crank_angle0))
crank.EnableCollision(False)
sys.AddBody(crank)

crank_disc = chrono.ChVisualShapeCylinder(0.16, 0.08)
crank_disc.SetColor(chrono.ChColor(0.12, 0.38, 0.85))
crank.AddVisualShape(crank_disc)
crank_arm = chrono.ChVisualShapeCylinder(0.035, crank_radius)
crank_arm.SetColor(chrono.ChColor(0.10, 0.20, 0.75))
crank.AddVisualShape(
    crank_arm,
    chrono.ChFramed(chrono.ChVector3d(crank_radius * 0.5, 0.0, 0.0), chrono.QuatFromAngleY(chrono.CH_PI_2)),
)
crank_pin_vis = chrono.ChVisualShapeSphere(0.055)
crank_pin_vis.SetColor(chrono.ChColor(0.9, 0.2, 0.1))
crank.AddVisualShape(crank_pin_vis, chrono.ChFramed(chrono.ChVector3d(crank_radius, 0.0, 0.0), chrono.QUNIT))

rod = chrono.ChBody()
rod.SetName("connecting_rod")
rod.SetMass(1.5)
rod.SetInertiaXX(chrono.ChVector3d(0.15, 0.15, 0.15))
rod.SetPos(rod_center0)
rod.SetRot(chrono.QuatFromAngleZ(rod_angle0))
rod.EnableCollision(False)
sys.AddBody(rod)

rod_shape = chrono.ChVisualShapeCylinder(0.045, rod_length)
rod_shape.SetColor(chrono.ChColor(0.85, 0.55, 0.10))
rod.AddVisualShape(rod_shape, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))

piston = chrono.ChBody()
piston.SetName("piston")
piston.SetMass(2.0)
piston.SetInertiaXX(chrono.ChVector3d(0.08, 0.08, 0.08))
piston.SetPos(piston_pos0)
piston.SetRot(chrono.QUNIT)
piston.EnableCollision(False)
sys.AddBody(piston)

piston_shape = chrono.ChVisualShapeBox(0.28, 0.18, 0.14)
piston_shape.SetColor(chrono.ChColor(0.25, 0.75, 0.25))
piston.AddVisualShape(piston_shape)


# === Joints / constraints === motorized crank, spherical pins, planar piston guide
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(crank, floor, chrono.ChFramed(chrono.ChVector3d(0.0, 0.0, 0.0), chrono.QUNIT))
motor.SetSpeedFunction(chrono.ChFunctionConst(crank_speed))
sys.AddLink(motor)

crank_rod = chrono.ChLinkLockSpherical()
crank_rod.Initialize(
    crank,
    rod,
    True,
    chrono.ChFramed(chrono.ChVector3d(crank_radius, 0.0, 0.0), chrono.QUNIT),
    chrono.ChFramed(chrono.ChVector3d(-rod_length * 0.5, 0.0, 0.0), chrono.QUNIT),
)
sys.AddLink(crank_rod)

rod_piston = chrono.ChLinkLockSpherical()
rod_piston.Initialize(
    rod,
    piston,
    True,
    chrono.ChFramed(chrono.ChVector3d(rod_length * 0.5, 0.0, 0.0), chrono.QUNIT),
    chrono.ChFramed(chrono.ChVector3d(0.0, 0.0, 0.0), chrono.QUNIT),
)
sys.AddLink(rod_piston)

piston_plane = chrono.ChLinkLockPlanar()
piston_plane.Initialize(piston, floor, chrono.ChFramed(piston_pos0, chrono.QUNIT))
sys.AddLink(piston_plane)


# === Visualization === Irrlicht window initialized before scene elements
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Slider-Crank with Spherical Pins and Planar Piston")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(1.4, -2.4, 1.2), chrono.ChVector3d(0.8, 0.0, 0.0))
vis.AddTypicalLights()
vis.AddGrid(
    0.2,
    0.2,
    14,
    10,
    chrono.ChCoordsysd(chrono.ChVector3d(0.8, 0.0, -0.09), chrono.QUNIT),
    chrono.ChColor(0.45, 0.45, 0.45),
)

crank_body = crank  # cache: fetched once for the main loop
rod_body = rod  # cache: fetched once for the main loop
piston_body = piston  # cache: fetched once for the main loop


# === Main loop === render once per frame and advance the pure MBS dynamics

while vis.Run() and sys.GetChTime() < sim_end:
    try:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            sys.DoStepDynamics(time_step)
            if sys.GetChTime() >= sim_end:
                break
    except (RuntimeError, ValueError) as exc:  # solver divergence / invalid numeric state
        print(f"simulation failed: {exc}")
        raise
    finally:
        pass
