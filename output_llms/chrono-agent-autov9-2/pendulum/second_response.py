"""Single pendulum swinging under lunar gravity, modeled as a pure jointed MBS.

Model
-----
A rigid pendulum arm (a thin cylinder) hangs from a fixed ground anchor through a
spherical joint located at the anchor. The arm is given an explicit mass and
inertia tensor and an initial angular velocity, then released to swing freely
under reduced (moon) gravity acting along world -Y.

System
------
- ChSystemNSC (non-smooth contact system), but the scene is a PURE JOINTED
  mechanism: the only constraint is the spherical joint and there is NO contact
  or collision anywhere. Therefore no collision system is configured.
- Gravity = (0, -1.62, 0) m/s^2 (lunar surface gravity), so the vertical world
  axis is Y; the visualization uses a Y-up camera.

Bodies
------
- ground : fixed anchor body carrying a visual sphere (radius 2) that marks the
  spherical joint location.
- arm    : the pendulum arm, a cylinder of radius 0.1 and length 1.5, mass 2 kg,
  inertia tensor (0.4, 1.5, 1.5). Its top end coincides with the joint anchor.

Expected behavior
-----------------
Released with an initial angular velocity, the arm swings about the anchor like
a spherical pendulum. Under the weak lunar gravity the oscillation is slow and
the motion stays smooth and bounded (the arm never escapes the joint anchor).
"""

import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# === Constants === geometry / physics parameters (no bare literals downstream)
time_step = 1e-3                 # integration step [s]
sim_end = 10.0                   # simulation duration [s]
render_fps = 50.0                # review render cadence [frames/s]

arm_radius = 0.1                 # pendulum arm cylinder radius [m]
arm_length = 1.5                 # pendulum arm cylinder length / pendulum length [m]
arm_mass = 2.0                   # pendulum mass [kg]
arm_inertia = chrono.ChVector3d(0.4, 1.5, 1.5)   # principal inertia tensor [kg*m^2]
joint_sphere_radius = 2.0        # visual sphere marking the spherical joint [m]

moon_gravity = chrono.ChVector3d(0, -1.62, 0)    # lunar gravitational acceleration [m/s^2]
init_ang_vel = chrono.ChVector3d(0, 0, 2.0)      # initial angular velocity about world Z [rad/s]

# Derived geometry: the joint anchor sits at the origin; the arm hangs straight
# down so its center of mass is half a length below the anchor (precomputed once).
anchor_pos = chrono.ChVector3d(0, 0, 0)                       # precomputed once
arm_com_pos = chrono.ChVector3d(0, -arm_length / 2.0, 0)      # precomputed once
render_every = max(1, round(1.0 / (render_fps * time_step)))  # precomputed once

# === System & gravity === pure jointed MBS, lunar gravity along -Y, no collision
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(moon_gravity)
# No SetCollisionSystemType: this is a pure jointed mechanism with no contact.

# === Bodies === fixed anchor (with joint marker sphere) + dynamic pendulum arm
ground = chrono.ChBody()
ground.SetFixed(True)
ground.SetPos(anchor_pos)
sys.AddBody(ground)

# Visual sphere (radius 2) marks the spherical joint location on the anchor.
# It is rendered semi-transparent so the pendulum arm, which swings inside this
# large radius-2 marker, stays visible through it.
joint_marker = chrono.ChVisualShapeSphere(joint_sphere_radius)
marker_mat = chrono.ChVisualMaterial()
marker_mat.SetDiffuseColor(chrono.ChColor(0.2, 0.4, 0.8))
marker_mat.SetOpacity(0.25)          # translucent so the inner arm remains visible
joint_marker.SetMaterial(0, marker_mat)
ground.AddVisualShape(joint_marker, chrono.ChFramed(anchor_pos, chrono.QUNIT))

# Pendulum arm: manual ChBody so mass + inertia tensor are set exactly as required.
arm = chrono.ChBody()
arm.SetMass(arm_mass)
arm.SetInertiaXX(arm_inertia)
arm.SetPos(arm_com_pos)
arm.EnableCollision(False)           # pure jointed mechanism: no contact geometry
arm.SetAngVelParent(init_ang_vel)    # release with an initial angular velocity
sys.AddBody(arm)

# Arm visual cylinder (radius 0.1, height 1.5). The cylinder's default axis is the
# body-local Z; rotate it to body-local Y so it spans the vertical arm length.
arm_cyl = chrono.ChVisualShapeCylinder(arm_radius, arm_length)
arm_cyl.SetColor(chrono.ChColor(0.8, 0.3, 0.2))
arm.AddVisualShape(arm_cyl, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(chrono.CH_PI_2)))

# === Joints / constraints === spherical joint connecting the arm to the ground anchor
# The connection point is the anchor at the arm's TOP end, expressed in ground-local
# coordinates (ground sits at the origin, so the anchor is local VNULL on ground).
spherical = chrono.ChLinkLockSpherical()
spherical.Initialize(arm, ground, chrono.ChFramed(anchor_pos, chrono.QUNIT))
sys.AddLink(spherical)

# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)   # gravity is along -Y -> Y-up view
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Single Pendulum on the Moon (spherical joint)")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, -0.75, 9), chrono.ChVector3d(0, -0.75, 0))
vis.AddTypicalLights()
vis.AddGrid(0.5, 0.5, 24, 24,
            chrono.ChCoordsysd(chrono.ChVector3d(0, -joint_sphere_radius - 0.5, 0),
                               chrono.QuatFromAngleX(chrono.CH_PI_2)),
            chrono.ChColor(0.4, 0.4, 0.4))   # ground reference grid below the pendulum

# === Main loop === real-time render-cadence loop; physics advanced in inner batch
try:
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            sys.DoStepDynamics(time_step)
            if sys.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / invalid state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass

# === Post-processing === assemble review video + plot, then clean up frames
