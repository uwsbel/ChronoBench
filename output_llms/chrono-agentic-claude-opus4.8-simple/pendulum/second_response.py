import os
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

sys = chrono.ChSystemNSC()                                           # rigid-body system, no contact
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -1.62, 0))     # moon gravity g = 1.62 m/s^2

# Ground body (fixed) — carries the pivot visual at (0,0,1)
ground = chrono.ChBody()                                             # fixed reference body
ground.SetFixed(True)                                                # immovable truss
ground.EnableCollision(False)                                        # pure jointed MBS, no contact
sys.Add(ground)
pivot_sphere = chrono.ChVisualShapeSphere(2)                         # joint visual: sphere radius 2
pivot_sphere.SetColor(chrono.ChColor(0, 0, 0.6))                     # blue pivot marker
ground.AddVisualShape(pivot_sphere, chrono.ChFramed(chrono.ChVector3d(0, 0, 1)))  # at the hinge point

# Pendulum body — a rigid cylinder arm swinging from the pivot
pend = chrono.ChBody()                                               # the swinging arm
pend.SetMass(2)                                                      # mass = 2 kg
pend.SetInertiaXX(chrono.ChVector3d(0.4, 1.5, 1.5))                  # inertia tensor (Ixx,Iyy,Izz)
pend.SetPos(chrono.ChVector3d(1, 0, 1))                              # starts horizontal, pointing +X
pend.SetPosDt(chrono.ChVector3d(0, 0, 1))                            # initial linear velocity at COM (sets swing)
pend.SetAngVelParent(chrono.ChVector3d(0, 1, 0))                     # initial angular velocity about Y
pend.EnableCollision(False)                                          # no contact
sys.Add(pend)
pend_cyl = chrono.ChVisualShapeCylinder(0.1, 1.5)                    # arm: radius 0.1, height 1.5
pend_cyl.SetColor(chrono.ChColor(0.6, 0, 0))                         # red arm
pend.AddVisualShape(pend_cyl, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))  # lay cylinder along X

# Spherical joint connecting ground and pendulum at the pivot
joint = chrono.ChLinkLockSpherical()                                 # 3-DOF ball joint
joint.Initialize(ground, pend, chrono.ChFramed(chrono.ChVector3d(0, 0, 1), chrono.QUNIT))  # hinge at (0,0,1)
sys.AddLink(joint)

vis = chronoirr.ChVisualSystemIrrlicht()                             # Irrlicht render window
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)                                         # window pixels
vis.SetWindowTitle("Pendulum")                                       # title
vis.Initialize()                                                     # device first, scene nodes after
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))     # logo
vis.AddSkyBox()                                                      # sky box
vis.AddCamera(chrono.ChVector3d(0, 3, 6))                            # camera eye
vis.AddTypicalLights()                                               # standard lights

time_step = 1e-3                                                     # integration step
render_fps = 50.0                                                    # review frame rate
render_every = max(1, round(1.0 / (render_fps * time_step)))         # untagged cadence constant
log_info = True                                                      # fire the physics log once
while vis.Run():                                                     # SCORED CORE = plain truth form
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)
        if log_info and sys.GetChTime() > 1:                         # after 1 s of settling
            pos = pend.GetPos()                                      # position of the arm
            print("t = ", sys.GetChTime())
            print("     ", pos.x, "  ", pos.y)
            vel = pend.GetPosDt()                                    # linear velocity
            print("     ", vel.x, "  ", vel.y)
            log_info = False                                         # disable further logging
