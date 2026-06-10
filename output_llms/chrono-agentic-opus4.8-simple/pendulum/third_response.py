import os
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

sys = chrono.ChSystemNSC()                                            # NSC system, no contact
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))      # g = 9.81 down (Z-up)

# ground / fixed truss anchoring the first hinge
ground = chrono.ChBody()                                              # fixed reference body
ground.SetFixed(True)                                                 # truss is immovable
sys.AddBody(ground)

arm_L = 1.0                                                           # arm length (m)
arm_r = 0.05                                                          # arm radius (m)
pivot0 = chrono.ChVector3d(0, 0, 0)                                   # first hinge at origin

# first pendulum arm: origin at its own center, hangs along -X from the pivot
arm1 = chrono.ChBodyEasyCylinder(chrono.ChAxis_X, arm_r, arm_L, 1000, True, False)  # density, visual, no collision
arm1.SetPos(chrono.ChVector3d(arm_L / 2, 0, 0))                       # center is half a length out along +X
sys.AddBody(arm1)

# second pendulum arm hinged to the far end of the first arm
arm2 = chrono.ChBodyEasyCylinder(chrono.ChAxis_X, arm_r, arm_L, 1000, True, False)  # same rod geometry
arm2.SetPos(chrono.ChVector3d(arm_L + arm_L / 2, 0, 0))              # near end lands on arm1 far end
sys.AddBody(arm2)

# hinge axis is world +Y (swing in the XZ plane under -Z gravity) -> map local +Z onto +Y
q_hinge_y = chrono.QuatFromAngleAxis(chrono.CH_PI_2, chrono.VECT_X)   # local +Z -> world +Y

# hinge 1: arm1 to ground at the origin pivot
hinge1 = chrono.ChLinkLockRevolute()                                  # revolute pin to ground
hinge1.Initialize(arm1, ground, chrono.ChFramed(pivot0, q_hinge_y))  # hinge about world +Y
sys.AddLink(hinge1)

# hinge 2: arm2 near end to arm1 far end (body-local frames, 5-arg form)
hinge2 = chrono.ChLinkLockRevolute()                                 # revolute pin between arms
hinge2.Initialize(
    arm1, arm2, True,
    chrono.ChFramed(chrono.ChVector3d(+arm_L / 2, 0, 0), q_hinge_y),  # arm1 FAR end (local +X)
    chrono.ChFramed(chrono.ChVector3d(-arm_L / 2, 0, 0), q_hinge_y),  # arm2 NEAR end (local -X)
)
sys.AddLink(hinge2)

vis = chronoirr.ChVisualSystemIrrlicht()                             # Irrlicht render window
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)                    # Z-up camera (before Initialize)
vis.SetWindowSize(1280, 720)                                         # window resolution
vis.SetWindowTitle("Double Pendulum")                               # window title
vis.Initialize()                                                    # create device FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))    # logo (after Initialize)
vis.AddSkyBox()                                                      # sky box
vis.AddCamera(chrono.ChVector3d(0, -5, -1), chrono.ChVector3d(0, 0, -1))  # eye, look-at pivot
vis.AddTypicalLights()                                              # standard two-light setup

time_step = 1e-3                                                     # integration step (s)
sim_end = 10.0                                                       # stop time (s)
render_fps = 50.0                                                    # review frame rate
render_every = max(1, round(1.0 / (render_fps * time_step)))        # untagged cadence constant
log_info = True                                                     # fire the physics log once
while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)                              # advance one step
        if log_info and sys.GetChTime() > 1:                       # after 1 s of swinging
            pos = arm2.GetPos()                                    # position of the second arm
            print("t = ", sys.GetChTime())
            print("     ", pos.x, "  ", pos.z)
            vel = arm2.GetPosDt()                                  # linear velocity
            print("     ", vel.x, "  ", vel.z)
            log_info = False                                       # disable further logging
        if sys.GetChTime() >= sim_end:
            break
