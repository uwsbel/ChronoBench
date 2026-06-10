import os
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

sys = chrono.ChSystemNSC()                                          # rigid-body NSC system
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))    # g = 9.81 down (Y-up)

# --- geometry / inertial parameters (planar double pendulum in the XY plane) ---
arm_len = 2.0          # length of each pendulum arm [m]
arm_rad = 0.05         # visual rod radius [m]
arm_mass = 1.0         # mass of each arm [kg]
# slender-rod inertia about its center, perpendicular axis: I = m L^2 / 12
I_perp = arm_mass * arm_len * arm_len / 12.0
I_axial = 0.5 * arm_mass * arm_rad * arm_rad

# --- ground / pivot anchor (fixed) ---
ground = chrono.ChBody()                                            # fixed reference body
ground.SetFixed(True)                                              # immovable pivot anchor
ground.SetName("ground")
sys.AddBody(ground)
pivot_pos = chrono.ChVector3d(0, 0, 0)                             # world hinge of first arm
# small visual pin at the pivot so the support is visible
pin_vis = chrono.ChVisualShapeCylinder(0.08, 0.2)                  # R, height
pin_vis.SetColor(chrono.ChColor(0.3, 0.3, 0.3))
ground.AddVisualShape(pin_vis, chrono.ChFramed(pivot_pos, chrono.QUNIT))  # pin axis along Z (out of plane)

# --- arm 1: hinged to ground at the origin, raised to horizontal +X so the system swings ---
arm1 = chrono.ChBody()                                             # first pendulum arm
arm1.SetMass(arm_mass)                                             # mass of arm 1
arm1.SetInertiaXX(chrono.ChVector3d(I_perp, I_perp, I_axial))     # slender-rod inertia
arm1.SetPos(chrono.ChVector3d(arm_len / 2, 0, 0))                 # COM at midpoint, arm along +X
arm1.SetRot(chrono.QUNIT)                                         # body-local X = world +X (horizontal)
arm1.EnableCollision(False)                                        # pure jointed MBS, no contact
sys.AddBody(arm1)
arm1_vis = chrono.ChVisualShapeCylinder(arm_rad, arm_len)         # rod visual, default axis local Z
arm1_vis.SetColor(chrono.ChColor(0.8, 0.2, 0.2))                  # red arm
arm1.AddVisualShape(arm1_vis, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))  # Z->X

# --- arm 2: hinged to the FAR end of arm 1, also raised horizontal along +X initially ---
arm2 = chrono.ChBody()                                             # second pendulum arm
arm2.SetMass(arm_mass)                                             # mass of arm 2
arm2.SetInertiaXX(chrono.ChVector3d(I_perp, I_perp, I_axial))     # slender-rod inertia
arm2.SetPos(chrono.ChVector3d(arm_len + arm_len / 2, 0, 0))       # COM beyond arm1's far end, along +X
arm2.SetRot(chrono.QUNIT)                                         # body-local X = world +X (horizontal)
arm2.EnableCollision(False)                                        # no contact
sys.AddBody(arm2)
arm2_vis = chrono.ChVisualShapeCylinder(arm_rad, arm_len)         # rod visual
arm2_vis.SetColor(chrono.ChColor(0.2, 0.2, 0.8))                  # blue arm
arm2.AddVisualShape(arm2_vis, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))  # Z->X

# bob masses at the end of each arm (visual emphasis of the two links)
bob1 = chrono.ChVisualShapeSphere(0.12)                           # small bob at arm1 far end
bob1.SetColor(chrono.ChColor(0.9, 0.5, 0.1))
arm1.AddVisualShape(bob1, chrono.ChFramed(chrono.ChVector3d(arm_len / 2, 0, 0), chrono.QUNIT))  # far end local +X
bob2 = chrono.ChVisualShapeSphere(0.12)                           # small bob at arm2 far end
bob2.SetColor(chrono.ChColor(0.9, 0.5, 0.1))
arm2.AddVisualShape(bob2, chrono.ChFramed(chrono.ChVector3d(arm_len / 2, 0, 0), chrono.QUNIT))

# --- hinge 1: arm1 to ground at the world origin; hinge axis = world +Z (out of XY plane) ---
hinge1 = chrono.ChLinkLockRevolute()                              # first revolute pivot
hinge1.Initialize(arm1, ground, chrono.ChFramed(pivot_pos, chrono.QUNIT))  # local +Z hinge = world +Z
sys.AddLink(hinge1)

# --- hinge 2: arm2's near end to arm1's far end; body-local frames so each marker is on its body ---
hinge2 = chrono.ChLinkLockRevolute()                             # second revolute joint
hinge2.Initialize(
    arm1, arm2, True,                                            # frames in each body's local coords
    chrono.ChFramed(chrono.ChVector3d(+arm_len / 2, 0, 0), chrono.QUNIT),  # arm1 FAR end (local +X)
    chrono.ChFramed(chrono.ChVector3d(-arm_len / 2, 0, 0), chrono.QUNIT),  # arm2 NEAR end (local -X)
)
sys.AddLink(hinge2)

# --- Irrlicht visualization (Initialize first, then scene elements; NO grid) ---
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)                                            # bind the physical system
vis.SetWindowSize(1280, 720)                                    # render window size
vis.SetWindowTitle("Double Pendulum")                          # window title
vis.Initialize()                                                # create device BEFORE scene nodes
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()                                                 # standard sky box
vis.AddCamera(chrono.ChVector3d(0, -1, 10), chrono.ChVector3d(0, -1, 0))  # face the XY swing plane
vis.AddTypicalLights()                                          # standard two-light setup

time_step = 1e-3                                                # integration step [s]
sim_end = 20.0                                                  # total simulated time [s]
render_fps = 50.0                                               # frames per second for review
render_every = max(1, round(1.0 / (render_fps * time_step)))   # physics steps per rendered frame
log_info = True                                                # fire the physics log once
while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)                          # advance one step
        if log_info and sys.GetChTime() > 1:                   # after 1 s of swinging
            p1 = arm1.GetPos()                                 # arm1 COM position
            p2 = arm2.GetPos()                                 # arm2 COM position
            print("t = ", sys.GetChTime())
            print("     ", p1.x, "  ", p1.y)
            print("     ", p2.x, "  ", p2.y)
            v2 = arm2.GetPosDt()                               # arm2 linear velocity
            print("     ", v2.x, "  ", v2.y)
            log_info = False                                   # disable further logging
        if sys.GetChTime() >= sim_end:
            break
