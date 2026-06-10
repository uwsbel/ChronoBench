"""Simple pendulum simulation (PyChrono, Irrlicht visualization).

Models a single rigid pendulum hinged to a fixed ground body by a revolute
joint. The system is a ChSystemNSC with gravity along -Y. The pendulum is a
slender rod (modeled as a manual ChBody with an explicit mass and inertia)
whose upper end is pinned at a fixed pivot and which swings freely in the XY
plane about a hinge aligned with the world +Z axis.

Bodies:
  - ground  : fixed reference body carrying a visible support post + pivot pin.
  - pendulum: dynamic rod body; its top end coincides with the pivot.

This is a PURE JOINTED multi-body system with NO contact/collision, so no
collision system is configured (matching the idiomatic bare-pendulum setup).

Expected behavior: released horizontally, the pendulum swings down under
gravity, oscillating about the vertical with the fixed pivot acting as the
rotation axis. Position and velocity of the pendulum are logged periodically.
"""

import os
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# === Constants === geometry / physics parameters and derived pivot geometry
time_step = 1e-3          # integration step [s]
sim_end = 10.0            # total simulated time [s]
render_fps = 50.0         # review-render cadence [frames/s]

rod_length = 1.0          # pendulum rod length [m]
rod_radius = 0.03         # pendulum rod visual radius [m]
rod_mass = 1.0            # pendulum mass [kg]
pivot_pos = chrono.ChVector3d(0, 0, 0)   # world pivot (hinge) location

# Rod modeled with origin at its center; released horizontally along +X, so the
# center sits half a length out from the pivot and the top end meets the pivot.
rod_center = chrono.ChVector3d(pivot_pos.x + rod_length / 2.0, pivot_pos.y, pivot_pos.z)
# Slender-rod inertia about its center (transverse axes); axial term tiny.
inertia_transverse = (1.0 / 12.0) * rod_mass * rod_length * rod_length   # precomputed once
inertia_axial = 0.5 * rod_mass * rod_radius * rod_radius                 # precomputed once
render_every = max(1, round(1.0 / (render_fps * time_step)))            # precomputed once

# === System & gravity === single NSC system, gravity along -Y (XY swing plane)
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

# === Bodies === fixed ground (support post + pivot pin) + dynamic pendulum rod
ground = chrono.ChBody()
ground.SetFixed(True)
ground.SetName("ground")
sys.AddBody(ground)
# Visible vertical support post rising from below to the pivot, so the fixed
# anchor reads clearly in the rendered scene.
post_height = 1.4
post_vis = chrono.ChVisualShapeBox(0.08, post_height, 0.08)
post_vis.SetColor(chrono.ChColor(0.25, 0.25, 0.25))
ground.AddVisualShape(post_vis, chrono.ChFramed(
    chrono.ChVector3d(pivot_pos.x, pivot_pos.y - post_height / 2.0, pivot_pos.z), chrono.QUNIT))
# Pivot pin: short cylinder along world Z so the hinge axis is visible.
pin_vis = chrono.ChVisualShapeCylinder(rod_radius * 1.6, rod_radius * 6.0)
pin_vis.SetColor(chrono.ChColor(0.9, 0.9, 0.2))
ground.AddVisualShape(pin_vis, chrono.ChFramed(pivot_pos, chrono.QUNIT))

pendulum = chrono.ChBody()
pendulum.SetName("pendulum")
pendulum.SetMass(rod_mass)
pendulum.SetInertiaXX(chrono.ChVector3d(inertia_axial, inertia_transverse, inertia_transverse))
pendulum.SetPos(rod_center)
pendulum.EnableCollision(False)   # pure jointed MBS: no contact
sys.AddBody(pendulum)
# Rod visual: cylinder default axis is body-local Z; rotate Z->X so it lies along +X.
rod_vis = chrono.ChVisualShapeCylinder(rod_radius, rod_length)
rod_vis.SetColor(chrono.ChColor(0.8, 0.2, 0.2))
pendulum.AddVisualShape(rod_vis, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))
# Bob marker at the free (far) end of the rod for a clear visual.
bob_vis = chrono.ChVisualShapeSphere(rod_radius * 2.0)
bob_vis.SetColor(chrono.ChColor(0.2, 0.2, 0.8))
pendulum.AddVisualShape(bob_vis, chrono.ChFramed(chrono.ChVector3d(rod_length / 2.0, 0, 0), chrono.QUNIT))

# === Joints / constraints === revolute hinge: pendulum to ground about world +Z
# XY swing plane with gravity along -Y -> hinge axis is world +Z, so joint local
# +Z = world +Z and the frame rotation is identity (QUNIT).
hinge = chrono.ChLinkLockRevolute()
hinge.Initialize(pendulum, ground, chrono.ChFramed(pivot_pos, chrono.QUNIT))
sys.AddLink(hinge)

# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)   # gravity along -Y -> Y is up
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Simple Pendulum")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.2, -0.4, 3.2), chrono.ChVector3d(0.2, -0.4, 0))
vis.AddTypicalLights()
vis.AddGrid(0.5, 0.5, 24, 24,
            chrono.ChCoordsysd(chrono.ChVector3d(0, -1.4, 0),
                               chrono.QuatFromAngleX(chrono.CH_PI_2)),
            chrono.ChColor(0.4, 0.4, 0.4))   # ground reference grid in the XZ plane


# === Main loop === render-cadence outer loop; physics advanced in inner batch
frame = 0
pend = pendulum          # cache: fetched once, reused every step
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
