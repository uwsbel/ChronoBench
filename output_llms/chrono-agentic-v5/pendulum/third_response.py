"""Double pendulum simulation (PyChrono, NSC, Irrlicht).

Models a planar double pendulum in the XY plane under gravity along -Y:
  * a fixed ground body carrying a short visual pivot cylinder,
  * pend_1 — first arm, hinged to ground by a revolute joint about world +Z,
  * pend_2 — second arm, hinged to the far end of pend_1 by a second revolute
    joint, so the two arms swing independently and chaotically.

System: ChSystemNSC (pure jointed multi-body, no contact/collision). Expected
behavior: both arms released horizontally fall and swing freely, exchanging
energy with chaotic, independent motion of the two links.
"""

import os
import math

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# === Constants === geometry / physics (no bare position literals downstream)
TIME_STEP = 1e-2          # integration step [s]
SIM_END = 20.0            # bounded recording horizon [s]
RENDER_FPS = 50.0         # review render cadence [frames/s]

ARM_LEN = 2.0             # full length of each pendulum arm [m]
ARM_RADIUS = 0.2          # visual cylinder radius of each arm [m]
ARM_MASS = 1.0            # mass of each arm [kg]
ARM_INERTIA = chrono.ChVector3d(0.2, 1, 1)   # arm inertia tensor diag [kg m^2]

PIVOT = chrono.ChVector3d(0, 0, 1)           # ground hinge of the first arm
PEND1_POS = chrono.ChVector3d(1, 0, 1)       # first arm center (midpoint)
HINGE2 = chrono.ChVector3d(2, 0, 1)          # far end of arm 1 / near end of arm 2
PEND2_POS = chrono.ChVector3d(3, 0, 1)       # second arm center (midpoint)

RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))   # precomputed once

# === System & gravity === pure jointed MBS, gravity along -Y (XY swing plane)
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

# === Bodies === fixed ground (pivot post) + two free arm links
ground = chrono.ChBody()
ground.SetFixed(True)
ground.EnableCollision(False)
sys.Add(ground)

pivot_cyl = chrono.ChVisualShapeCylinder(ARM_RADIUS, 0.4)   # short hinge post
ground.AddVisualShape(pivot_cyl, chrono.ChFramed(PIVOT))

pend_1 = chrono.ChBody()
pend_1.SetFixed(False)
pend_1.EnableCollision(False)
pend_1.SetMass(ARM_MASS)
pend_1.SetInertiaXX(ARM_INERTIA)
pend_1.SetPos(PEND1_POS)
sys.AddBody(pend_1)

cyl_1 = chrono.ChVisualShapeCylinder(ARM_RADIUS, ARM_LEN)   # arm visual along local X
cyl_1.SetColor(chrono.ChColor(0.6, 0, 0))
pend_1.AddVisualShape(cyl_1, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))

pend_2 = chrono.ChBody()
pend_2.SetFixed(False)
pend_2.EnableCollision(False)
pend_2.SetMass(ARM_MASS)
pend_2.SetInertiaXX(ARM_INERTIA)
pend_2.SetPos(PEND2_POS)
sys.AddBody(pend_2)

cyl_2 = chrono.ChVisualShapeCylinder(ARM_RADIUS, ARM_LEN)
cyl_2.SetColor(chrono.ChColor(0, 0, 0.6))
pend_2.AddVisualShape(cyl_2, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))

# === Joints / constraints === two revolutes (hinge axis = world +Z, normal to XY)
rev_1 = chrono.ChLinkLockRevolute()
rev_1.Initialize(ground, pend_1, chrono.ChFramed(PIVOT, chrono.QUNIT))
sys.AddLink(rev_1)

rev_2 = chrono.ChLinkLockRevolute()
rev_2.Initialize(pend_1, pend_2, chrono.ChFramed(HINGE2, chrono.QUNIT))
sys.AddLink(rev_2)

# === Visualization === full Irrlicht scene: window + sky + camera + lights
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Double Pendulum Simulation')
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)   # gravity along -Y
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 3, 6), chrono.ChVector3d(2, 0, 1))
vis.AddTypicalLights()

# === Main loop === render-cadence outer loop, batch physics between frames
os.makedirs("cam", exist_ok=True)   # guard against missing output dir
try:

    frame = 0
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(RENDER_EVERY):
            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / invalid state
    import traceback
    traceback.print_exc()
    raise
