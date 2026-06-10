"""
Slider-Crank Mechanism Simulation (PyChrono 9.0.x, NSC, Irrlicht)

Models a classical crank-slider linkage:
  - Floor/truss: fixed rigid body (the ground reference)
  - Crankshaft: rotated by a constant-speed motor attached to the truss
  - Connecting rod: links crank-pin to wrist-pin on the piston
  - Piston: slides horizontally along a prismatic guide fixed to the truss

Topology (ground-truth):
  crank <-> truss  : ChLinkMotorRotationSpeed (full motor-link, no extra revolute)
  crank <-> rod    : ChLinkLockRevolute at crank-pin
  rod   <-> piston : ChLinkLockRevolute at wrist-pin
  piston <-> truss : ChLinkLockPrismatic (horizontal sliding guide)

System type: ChSystemNSC (pure jointed MBS, no collision/contact)
Gravity: Y-up (0, -9.81, 0); mechanism is in the XY plane
Expected: piston oscillates sinusoidally as the crank rotates at constant speed.
"""

import math
import os
import csv
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# === Named constants ===
CRANK_LEN   = 0.5      # half-length of crank from pivot to crank-pin [m]
ROD_LEN     = 1.6      # connecting rod length [m]
CRANK_SPEED = math.pi  # crankshaft angular speed [rad/s] (0.5 rev/s)

TIME_STEP   = 1e-3     # physics time-step [s]
SIM_END     = 10.0     # simulation end time [s]
RENDER_FPS  = 50.0
render_every = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

# Derived geometry (precomputed once)
CRANK_PIVOT = chrono.ChVector3d(0.0, 0.0, 0.0)          # crankshaft pivot on truss
CRANK_PIN   = chrono.ChVector3d(CRANK_LEN, 0.0, 0.0)    # crank COM midpoint = 0..crankpin, midpoint
CRANK_COM   = chrono.ChVector3d(CRANK_LEN / 2.0, 0.0, 0.0)  # crank body COM
ROD_COM     = chrono.ChVector3d(CRANK_LEN + ROD_LEN / 2.0, 0.0, 0.0)  # rod body COM (horizontal at t=0)
PISTON_X    = CRANK_LEN + ROD_LEN                        # piston initial X position
PISTON_POS  = chrono.ChVector3d(PISTON_X, 0.0, 0.0)


# === System & gravity ===
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))
# Pure jointed MBS — no collision shapes, so SetCollisionSystemType is intentionally omitted.

# === Bodies ===

# Truss (floor / ground): fixed
truss = chrono.ChBody()
truss.SetFixed(True)
truss.SetName("truss")
truss_vis = chrono.ChVisualShapeBox(0.2, 0.2, 0.5)
truss_vis.SetColor(chrono.ChColor(0.5, 0.5, 0.5))
truss.AddVisualShape(truss_vis, chrono.ChFramed(CRANK_PIVOT))
sys.AddBody(truss)

# Crankshaft body: rotates in XY plane about CRANK_PIVOT
crank = chrono.ChBody()
crank.SetName("crank")
crank.SetMass(1.0)
crank.SetInertiaXX(chrono.ChVector3d(0.05, 0.05, 0.1))
crank.SetPos(CRANK_COM)
cyl_crank = chrono.ChVisualShapeCylinder(0.05, CRANK_LEN)
cyl_crank.SetColor(chrono.ChColor(0.8, 0.3, 0.1))
# Step 1: body-local X along world X (identity); Step 2: visual offset aligns cylinder Z -> X
crank.AddVisualShape(cyl_crank, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))
sys.AddBody(crank)

# Connecting rod: links crank-pin to wrist-pin (piston), initially horizontal
rod = chrono.ChBody()
rod.SetName("rod")
rod.SetMass(0.5)
rod.SetInertiaXX(chrono.ChVector3d(0.05, 0.5, 0.5))
rod.SetPos(ROD_COM)
cyl_rod = chrono.ChVisualShapeCylinder(0.04, ROD_LEN)
cyl_rod.SetColor(chrono.ChColor(0.2, 0.6, 0.8))
rod.AddVisualShape(cyl_rod, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))
sys.AddBody(rod)

# Piston: slides horizontally
piston = chrono.ChBody()
piston.SetName("piston")
piston.SetMass(1.0)
piston.SetInertiaXX(chrono.ChVector3d(0.1, 0.1, 0.1))
piston.SetPos(PISTON_POS)
piston_vis = chrono.ChVisualShapeBox(0.25, 0.25, 0.25)
piston_vis.SetColor(chrono.ChColor(0.1, 0.8, 0.2))
piston.AddVisualShape(piston_vis)
sys.AddBody(piston)

# === Joints / constraints ===

# Motor: crank spun at constant speed relative to truss (full motor-link — no extra revolute)
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(crank, truss, chrono.ChFramed(CRANK_PIVOT, chrono.QUNIT))
motor.SetSpeedFunction(chrono.ChFunctionConst(CRANK_SPEED))
sys.AddLink(motor)

# Revolute at crank-pin: connects crank FAR end to rod NEAR end (body-local form)
# crank COM at CRANK_LEN/2 from pivot, so crank-pin in crank-local = +CRANK_LEN/2 along X
# rod COM at (CRANK_LEN + ROD_LEN/2); rod near-end in rod-local = -ROD_LEN/2 along X
rev_crank_rod = chrono.ChLinkLockRevolute()
rev_crank_rod.Initialize(
    crank, rod, True,
    chrono.ChFramed(chrono.ChVector3d(CRANK_LEN / 2.0, 0.0, 0.0), chrono.QUNIT),  # far end of crank (local)
    chrono.ChFramed(chrono.ChVector3d(-ROD_LEN / 2.0, 0.0, 0.0), chrono.QUNIT),   # near end of rod (local)
)
sys.AddLink(rev_crank_rod)

# Revolute at wrist-pin: rod far end to piston
# rod far-end in rod-local = +ROD_LEN/2; piston COM = PISTON_POS, so piston-local = (0,0,0)
rev_rod_piston = chrono.ChLinkLockRevolute()
rev_rod_piston.Initialize(
    rod, piston, True,
    chrono.ChFramed(chrono.ChVector3d(ROD_LEN / 2.0, 0.0, 0.0), chrono.QUNIT),  # far end of rod (local)
    chrono.ChFramed(chrono.ChVector3d(0.0, 0.0, 0.0), chrono.QUNIT),            # piston center (local)
)
sys.AddLink(rev_rod_piston)

# Prismatic: piston slides along world X on the fixed truss
# ChLinkLockPrismatic uses frame local +Z as slide axis -> rotate Z to X
prism = chrono.ChLinkLockPrismatic()
prism.Initialize(piston, truss, chrono.ChFramed(PISTON_POS, chrono.Q_ROTATE_Z_TO_X))
sys.AddLink(prism)

# === Visualization ===  full Irrlicht scene: window + sky + camera + lights
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Slider-Crank Mechanism")
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.Initialize()                                              # Initialize FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(1.5, 1.5, 3.0), chrono.ChVector3d(1.5, 0.0, 0.0))
vis.AddTypicalLights()


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
except (RuntimeError, ValueError) as exc:   # solver divergence / bad body state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass  # no persistent writers to close in scored core
