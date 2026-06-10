import os
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# pendulum parameters
pend_length = 1.0          # pendulum arm length [m]
pend_mass = 1.0            # pendulum bob mass [kg]
pend_radius = 0.05         # bob visual sphere radius [m]
time_step = 1e-3           # integration time step [s]
sim_end = 10.0             # simulation end time [s]
render_fps = 50.0          # render frame rate [fps]

# system (pure jointed MBS — no collision needed)
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))       # Y-up, g down

# ground body (fixed support)
ground = chrono.ChBody()
ground.SetFixed(True)                                                   # immovable anchor
sys.AddBody(ground)

# pendulum arm body — modeled as a rod of given length and mass
pend = chrono.ChBody()
pend.SetMass(pend_mass)                                                 # 1 kg
# solid rod inertia: I_xx = I_yy = (1/12)*m*L^2, I_zz small
Ixx = (1.0 / 12.0) * pend_mass * pend_length ** 2                     # bending inertia
pend.SetInertiaXX(chrono.ChVector3d(Ixx, Ixx, Ixx))
# pivot at (0,0,0), COM at midpoint of arm hanging down
pend.SetPos(chrono.ChVector3d(0, -pend_length / 2.0, 0))              # hanging vertically
sys.AddBody(pend)

# visual shapes: rod cylinder + bob sphere
cyl = chrono.ChVisualShapeCylinder(0.02, pend_length)                  # thin rod
pend.AddVisualShape(cyl, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))
sph = chrono.ChVisualShapeSphere(pend_radius)                          # bob at bottom
pend.AddVisualShape(sph, chrono.ChFramed(chrono.ChVector3d(0, -pend_length / 2.0, 0)))

# revolute joint at pivot (0,0,0) — hinge about world Z (XY swing plane, gravity -Y)
# ChLinkLockRevolute local +Z is the hinge axis; QUNIT gives local +Z = world +Z
hinge = chrono.ChLinkLockRevolute()
hinge.Initialize(pend, ground, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
sys.AddLink(hinge)

# give initial angular kick so it actually swings
pend.SetAngVelParent(chrono.ChVector3d(0, 0, 1.5))                    # rad/s about Z

# Irrlicht visualization — Initialize FIRST, scene elements AFTER
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)                      # Y-up view
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Simple Pendulum")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 0, 3), chrono.ChVector3d(0, -0.5, 0))
vis.AddTypicalLights()

render_every = max(1, round(1.0 / (render_fps * time_step)))           # untagged cadence constant
log_info = True                                                        # one-shot physics log flag
while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene(); vis.Render(); vis.EndScene()
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)
        if log_info and sys.GetChTime() > 1:                          # after 1 s of settling
            pos = pend.GetPos()                                       # position of the pendulum bob
            print("t = ", sys.GetChTime())
            print("     ", pos.x, "  ", pos.y)
            vel = pend.GetPosDt()                                     # linear velocity
            print("     ", vel.x, "  ", vel.y)
            log_info = False                                          # disable further logging
        if sys.GetChTime() >= sim_end:
            break
