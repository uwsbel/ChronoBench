"""Single pendulum on the Moon, swinging from a spherical joint.

System type: ChSystemNSC (pure jointed multi-body, no contact/collision).
Bodies:
  - ground: a fixed reference body carrying a sphere visual that marks the pivot.
  - pendulum: a rod (cylinder visual) hinged to ground by a spherical joint, with
    a prescribed mass and inertia tensor and a non-zero initial angular velocity.
Gravity is the lunar value (0, -1.62, 0), Y-up. Expected behavior: the pendulum
swings/precesses about the pivot under the weak lunar gravity, the spherical joint
allowing 3-DOF rotation about the anchor while the initial spin sets it in motion.
"""

import os
import csv
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# === Named constants === geometry / mass / physics (no bare literals downstream)
TIME_STEP = 1e-3
SIM_END = 10.0
RENDER_FPS = 50.0

PIVOT = chrono.ChVector3d(0, 0, 0)        # world anchor of the spherical joint
JOINT_VIS_RADIUS = 2.0                    # sphere marking the joint
ROD_RADIUS = 0.1                          # pendulum cylinder radius
ROD_LENGTH = 1.5                          # pendulum cylinder height (its length)
PEND_MASS = 2.0                           # pendulum mass [kg]
PEND_INERTIA = chrono.ChVector3d(0.4, 1.5, 1.5)   # inertia tensor (Ixx,Iyy,Izz)
INIT_ANGVEL = chrono.ChVector3d(0, 0, 2.0)        # initial angular velocity [rad/s]
GRAVITY_MOON = chrono.ChVector3d(0, -1.62, 0)     # lunar gravitational acceleration

# Pendulum hangs below the pivot: COM at half its length down the -Y axis.
PEND_POS = chrono.ChVector3d(0, -ROD_LENGTH / 2.0, 0)   # precomputed once

# === System & gravity === pure jointed MBS (no contact -> no collision system)
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(GRAVITY_MOON)

# === Bodies === fixed ground (pivot marker) + the swinging pendulum rod
ground = chrono.ChBody()
ground.SetFixed(True)
ground.SetPos(PIVOT)
joint_sphere = chrono.ChVisualShapeSphere(JOINT_VIS_RADIUS)
joint_sphere.SetColor(chrono.ChColor(0.2, 0.5, 0.9))
ground.AddVisualShape(joint_sphere)
sys.AddBody(ground)

pendulum = chrono.ChBody()
pendulum.SetMass(PEND_MASS)
pendulum.SetInertiaXX(PEND_INERTIA)
pendulum.SetPos(PEND_POS)
pendulum.SetAngVelParent(INIT_ANGVEL)     # set the pendulum spinning at t = 0
rod = chrono.ChVisualShapeCylinder(ROD_RADIUS, ROD_LENGTH)
rod.SetColor(chrono.ChColor(0.8, 0.3, 0.2))
# Cylinder default axis is body-local Z; the rod hangs along Y -> rotate Z onto Y.
pendulum.AddVisualShape(rod, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(chrono.CH_PI_2)))
sys.AddBody(pendulum)

# === Joints / constraints === spherical joint pinning the rod top to the pivot
spherical = chrono.ChLinkLockSpherical()
spherical.Initialize(pendulum, ground, chrono.ChFramed(PIVOT))
sys.AddLink(spherical)

# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)   # gravity along -Y -> Y-up camera
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Spherical-joint pendulum on the Moon")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(6, 2, 8), chrono.ChVector3d(0, -1, 0))
vis.AddTypicalLights()

# === Main loop === real-time stepping; physics in an inner batch between frames
render_every = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))   # precomputed once
get_time = sys.GetChTime                                       # cache: getter reused every step

os.makedirs("cam", exist_ok=True)                             # guard against missing output dir


frame = 0
try:
    while vis.Run() and get_time() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            sys.DoStepDynamics(TIME_STEP)
            if get_time() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / invalid state
    import traceback
    traceback.print_exc()
    raise

# === Post-processing === assemble the review video + plot, then drop frame dirs
