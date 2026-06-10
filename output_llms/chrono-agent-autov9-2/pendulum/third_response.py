"""Double pendulum simulation (PyChrono 9.0.x, Irrlicht).

Models a planar double pendulum as a pure jointed multi-body system:
  * system type: ChSystemNSC (no contact — purely jointed mechanism)
  * bodies: a fixed ground anchor plus two uniform rigid rods (rod1, rod2)
  * joints: two revolute hinges — rod1 hinged to ground at the fixed pivot,
    rod2 hinged to the free (far) end of rod1 — so both rods swing freely and
    independently under gravity in the vertical X-Z plane.

Gravity acts along world -Z (Z-up convention). The swing plane is X-Z, so each
revolute hinge axis is world +Y (normal to the swing plane). With both rods
released from a raised horizontal pose, the expected behavior is the classic
chaotic, energy-conserving double-pendulum motion: the two rods swing with
coupled, non-periodic trajectories and no damping.
"""

import math

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr


# === Constants === geometry / physics parameters and derived positions
time_step = 1.0e-3            # integration step [s]
sim_end = 20.0               # simulation duration [s]
render_fps = 50.0            # review render cadence [frames/s]

rod_length = 1.0             # length of each rod [m]
rod_radius = 0.04            # visual rod radius [m]
rod_mass = 1.0               # mass of each rod [kg]
half_len = rod_length / 2.0  # precomputed once: rod half-length [m]

pivot_z = 2.0                # world Z of the fixed top pivot [m]

# Uniform-rod inertia about transverse axes (slender rod): I = m*L^2/12.
# About the rod's own long axis the inertia is tiny; use a small value.
inertia_trans = rod_mass * rod_length * rod_length / 12.0   # precomputed once
inertia_axial = 0.5 * rod_mass * rod_radius * rod_radius     # precomputed once

# Hinge axis: swing happens in the X-Z plane, so the physical hinge axis is
# world +Y. Map joint local +Z onto +Y by rotating +90 deg about world X.
q_hinge_y = chrono.QuatFromAngleAxis(chrono.CH_PI_2, chrono.VECT_X)  # precomputed once

# Initial pose: both rods released horizontally along +X (chaotic start).
pivot_pos = chrono.ChVector3d(0.0, 0.0, pivot_z)                 # fixed top pivot
rod1_center = chrono.ChVector3d(half_len, 0.0, pivot_z)          # rod1 COM
rod1_far_end = chrono.ChVector3d(rod_length, 0.0, pivot_z)       # rod1 free end
rod2_center = chrono.ChVector3d(rod_length + half_len, 0.0, pivot_z)  # rod2 COM

render_every = max(1, round(1.0 / (render_fps * time_step)))    # precomputed once


def make_rod(center_pos):
    """Build one uniform rod body, COM at center_pos, long axis along world +X."""
    body = chrono.ChBody()
    body.SetMass(rod_mass)
    body.SetInertiaXX(chrono.ChVector3d(inertia_axial, inertia_trans, inertia_trans))
    body.SetPos(center_pos)
    body.SetRot(chrono.QUNIT)            # body-local X already aligned with world X
    body.EnableCollision(False)          # pure jointed mechanism — no contact
    # Cylinder default axis is body-local Z; rotate Z->X so it spans the rod length.
    cyl = chrono.ChVisualShapeCylinder(rod_radius, rod_length)
    cyl.SetColor(chrono.ChColor(0.2, 0.4, 0.8))
    body.AddVisualShape(cyl, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))
    return body


# === System & gravity === pure jointed MBS, gravity along world -Z (Z-up)
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0.0, 0.0, -9.81))
# NOTE: no SetCollisionSystemType — this scene has no contact/collision at all.

# === Bodies === fixed ground anchor + two swinging rods
ground = chrono.ChBody()
ground.SetFixed(True)
ground.SetPos(pivot_pos)
ground.EnableCollision(False)
pin_vis = chrono.ChVisualShapeCylinder(rod_radius * 1.2, rod_radius * 4.0)
pin_vis.SetColor(chrono.ChColor(0.5, 0.5, 0.5))
# Pivot pin drawn along world Y (the hinge axis) for a visible support marker.
ground.AddVisualShape(pin_vis, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(chrono.CH_PI_2)))
sys.AddBody(ground)

rod1 = make_rod(rod1_center)
sys.AddBody(rod1)

rod2 = make_rod(rod2_center)
sys.AddBody(rod2)

# === Joints / constraints === two revolute hinges (axis world +Y, X-Z swing plane)
# Hinge 1: rod1 near end to ground at the fixed top pivot.
hinge1 = chrono.ChLinkLockRevolute()
hinge1.Initialize(rod1, ground, chrono.ChFramed(pivot_pos, q_hinge_y))
sys.AddLink(hinge1)

# Hinge 2: rod2 near end (-L/2) to rod1 far end (+L/2), via body-local frames so
# each marker is defined on its own body; both map to rod1_far_end in world space.
hinge2 = chrono.ChLinkLockRevolute()
hinge2.Initialize(
    rod2, rod1, True,
    chrono.ChFramed(chrono.ChVector3d(-half_len, 0.0, 0.0), q_hinge_y),  # rod2 near end
    chrono.ChFramed(chrono.ChVector3d(+half_len, 0.0, 0.0), q_hinge_y),  # rod1 far end
)
sys.AddLink(hinge2)

# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Double Pendulum")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.0, -6.0, 1.5), chrono.ChVector3d(1.0, 0.0, 1.0))
vis.AddTypicalLights()
vis.AddGrid(0.5, 0.5, 40, 40, chrono.ChCoordsysd(), chrono.ChColor(0.4, 0.4, 0.4))

# === Main loop === render-cadence outer loop; physics advanced in an inner batch
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
