"""Motor-driven slider-crank mechanism in a Y-up NSC multibody system.

The model contains a fixed truss/floor, a rotating crankshaft, a connecting
rod, and a piston constrained to a straight guide. A prescribed-speed motor
spins the crank at constant angular speed, converting rotation into piston
translation through revolute crank-pin and wrist-pin joints.
"""

import math
import traceback

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr


# === Constants === define direct mechanism dimensions and run cadence
TIME_STEP = 1e-3
SIM_END = 6.0
RENDER_FPS = 50.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

CRANK_RADIUS = 0.5
ROD_LENGTH = 1.5
CRANK_SPEED = 2.0 * math.pi
ROD_RADIUS = 0.035
CRANK_RADIUS_VIS = 0.04
CRANKSHAFT_RADIUS = 0.12
CRANKSHAFT_WIDTH = 0.12
PISTON_SIZE_X = 0.32
PISTON_SIZE_Y = 0.22
PISTON_SIZE_Z = 0.28
FLOOR_SIZE_X = 3.2
FLOOR_SIZE_Y = 0.08
FLOOR_SIZE_Z = 0.9

PIVOT = chrono.ChVector3d(0.0, 0.0, 0.0)
CRANK_CENTER = chrono.ChVector3d(CRANK_RADIUS / 2.0, 0.0, 0.0)
ROD_CENTER = chrono.ChVector3d(CRANK_RADIUS + ROD_LENGTH / 2.0, 0.0, 0.0)
PISTON_CENTER = chrono.ChVector3d(CRANK_RADIUS + ROD_LENGTH, 0.0, 0.0)


def set_visual_color(body, color):
    """Apply a color to all visual assets on a body."""
    shape = body.GetVisualShape(0)  # cache: factory-created primary visual shape
    shape.SetColor(color)


# === System & gravity === use a pure jointed NSC system without contact
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0.0, -9.81, 0.0))


# === Bodies === create the fixed truss, crankshaft, connecting rod, and piston
truss = chrono.ChBody()
truss.SetName("fixed_truss_floor")
truss.SetFixed(True)
truss.SetPos(chrono.ChVector3d(0.8, -0.12, 0.0))
floor_shape = chrono.ChVisualShapeBox(FLOOR_SIZE_X, FLOOR_SIZE_Y, FLOOR_SIZE_Z)
floor_shape.SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
truss.AddVisualShape(floor_shape)
sys.AddBody(truss)

crankshaft = chrono.ChBodyEasyCylinder(chrono.ChAxis_Z, CRANKSHAFT_RADIUS, CRANKSHAFT_WIDTH, 1000.0)
crankshaft.SetName("crankshaft")
crankshaft.SetMass(1.0)
crankshaft.SetInertiaXX(chrono.ChVector3d(0.02, 0.02, 0.02))
crankshaft.SetPos(PIVOT)
set_visual_color(crankshaft, chrono.ChColor(0.25, 0.25, 0.28))
sys.AddBody(crankshaft)

crank = chrono.ChBody()
crank.SetName("crank")
crank.SetMass(1.0)
crank.SetInertiaXX(chrono.ChVector3d(0.02, 0.02, 0.02))
crank.SetPos(CRANK_CENTER)
crank.SetRot(chrono.QUNIT)
crank_visual = chrono.ChVisualShapeCylinder(CRANK_RADIUS_VIS, CRANK_RADIUS)
crank_visual.SetColor(chrono.ChColor(0.85, 0.15, 0.12))
crank.AddVisualShape(crank_visual, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))
sys.AddBody(crank)

rod = chrono.ChBody()
rod.SetName("connecting_rod")
rod.SetMass(1.0)
rod.SetInertiaXX(chrono.ChVector3d(0.03, 0.03, 0.03))
rod.SetPos(ROD_CENTER)
rod.SetRot(chrono.QUNIT)
rod_visual = chrono.ChVisualShapeCylinder(ROD_RADIUS, ROD_LENGTH)
rod_visual.SetColor(chrono.ChColor(0.1, 0.35, 0.85))
rod.AddVisualShape(rod_visual, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))
sys.AddBody(rod)

piston = chrono.ChBody()
piston.SetName("piston")
piston.SetMass(1.0)
piston.SetInertiaXX(chrono.ChVector3d(0.02, 0.02, 0.02))
piston.SetPos(PISTON_CENTER)
piston.SetRot(chrono.QUNIT)
piston_shape = chrono.ChVisualShapeBox(PISTON_SIZE_X, PISTON_SIZE_Y, PISTON_SIZE_Z)
piston_shape.SetColor(chrono.ChColor(0.25, 0.75, 0.25))
piston.AddVisualShape(piston_shape)
sys.AddBody(piston)


# === Joints / constraints === enforce slider-crank topology with a speed motor
crank_to_shaft = chrono.ChLinkLockLock()
crank_to_shaft.Initialize(
    crank,
    crankshaft,
    True,
    chrono.ChFramed(chrono.ChVector3d(-CRANK_RADIUS / 2.0, 0.0, 0.0), chrono.QUNIT),
    chrono.ChFramed(chrono.VNULL, chrono.QUNIT),
)
sys.AddLink(crank_to_shaft)

motor = chrono.ChLinkMotorRotationSpeed()
motor.SetName("constant_speed_crank_motor")
motor.Initialize(crankshaft, truss, chrono.ChFramed(PIVOT, chrono.QUNIT))
motor.SetSpeedFunction(chrono.ChFunctionConst(CRANK_SPEED))
sys.AddLink(motor)

crank_pin = chrono.ChLinkLockRevolute()
crank_pin.SetName("crank_pin_revolute")
crank_pin.Initialize(
    crank,
    rod,
    True,
    chrono.ChFramed(chrono.ChVector3d(CRANK_RADIUS / 2.0, 0.0, 0.0), chrono.QUNIT),
    chrono.ChFramed(chrono.ChVector3d(-ROD_LENGTH / 2.0, 0.0, 0.0), chrono.QUNIT),
)
sys.AddLink(crank_pin)

wrist_pin = chrono.ChLinkLockRevolute()
wrist_pin.SetName("wrist_pin_revolute")
wrist_pin.Initialize(
    rod,
    piston,
    True,
    chrono.ChFramed(chrono.ChVector3d(ROD_LENGTH / 2.0, 0.0, 0.0), chrono.QUNIT),
    chrono.ChFramed(chrono.VNULL, chrono.QUNIT),
)
sys.AddLink(wrist_pin)

piston_guide = chrono.ChLinkLockPrismatic()
piston_guide.SetName("piston_ground_prismatic")
piston_guide.Initialize(piston, truss, chrono.ChFramed(PISTON_CENTER, chrono.Q_ROTATE_Z_TO_X))
sys.AddLink(piston_guide)


# === Visualization === configure Irrlicht window, camera, lights, logo, and grid
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("PyChrono slider-crank mechanism")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(1.0, 1.2, 3.0), chrono.ChVector3d(0.8, 0.0, 0.0))
vis.AddTypicalLights()
vis.AddGrid(
    0.2,
    0.2,
    18,
    10,
    chrono.ChCoordsysd(chrono.ChVector3d(0.8, -0.17, 0.0), chrono.QuatFromAngleX(chrono.CH_PI_2)),
    chrono.ChColor(0.45, 0.45, 0.45),
)

crank_body = crank  # cache: reused in the loop for crank-pin state
rod_body = rod  # cache: reused in the loop for rod state
piston_body = piston  # cache: reused in the loop for piston state


# === Main loop === render at a fixed cadence and advance the jointed system
frame = 0
try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(RENDER_EVERY):
            sim_time = sys.GetChTime()  # cache: current time reused for logging
            piston_pos = piston_body.GetPos()  # cache: piston pose reused in this step
            piston_vel = piston_body.GetPosDt()  # cache: piston velocity reused in this step
            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:  # solver divergence or invalid state
    traceback.print_exc()
    raise
finally:
    final_time = sys.GetChTime()  # cache: visible cleanup/final-state checkpoint


# === Post-processing === write review data and assemble the Irrlicht video

print(f"Simulation finished at t={final_time:.3f} s")
