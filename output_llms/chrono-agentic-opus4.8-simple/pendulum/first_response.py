import os
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

sys = chrono.ChSystemNSC()                                           # NSC rigid-body system
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))     # g = 9.81, Y-up

ground = chrono.ChBody()                                              # fixed reference body
ground.SetFixed(True)                                                # ground does not move
sys.AddBody(ground)

pin = chrono.ChVisualShapeCylinder(0.04, 0.2)                        # small pivot pin for the hinge
pin.SetColor(chrono.ChColor(0.3, 0.3, 0.3))                          # dark grey pin
ground.AddVisualShape(pin, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(chrono.CH_PI_2)))  # pin along world Z (hinge axis)

pend_len = 1.0                                                        # pendulum arm length [m]
pend_mass = 1.0                                                       # pendulum mass [kg]
pend_radius = 0.05                                                    # arm visual radius [m]
pivot = chrono.ChVector3d(0, 0, 0)                                   # hinge point at world origin
com = chrono.ChVector3d(pend_len, 0, 0)                              # arm center of mass, hanging out along +X

pendulum = chrono.ChBody()                                            # the swinging arm
pendulum.SetMass(pend_mass)                                          # total mass
Ixx = (1.0 / 12.0) * pend_mass * (2 * pend_len) ** 2                 # slender-rod inertia about transverse axes
pendulum.SetInertiaXX(chrono.ChVector3d(1e-3, Ixx, Ixx))            # inertia tensor (thin rod along X)
pendulum.SetPos(com)                                                 # place COM at half-swing position
sys.AddBody(pendulum)

arm = chrono.ChVisualShapeCylinder(pend_radius, 2 * pend_len)        # rod visual spanning the arm
arm.SetColor(chrono.ChColor(0.6, 0.2, 0.2))                         # red arm
pendulum.AddVisualShape(arm, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))  # cylinder local Z -> local X (arm direction)

bob = chrono.ChVisualShapeSphere(0.12)                              # bob at the far end
bob.SetColor(chrono.ChColor(0.2, 0.2, 0.6))                        # blue bob
pendulum.AddVisualShape(bob, chrono.ChFramed(chrono.ChVector3d(pend_len, 0, 0)))  # far end of the arm

revolute = chrono.ChLinkLockRevolute()                              # hinge: planar XY swing, hinge axis world +Z
revolute.Initialize(ground, pendulum, chrono.ChFramed(pivot, chrono.QUNIT))  # local +Z = world +Z
sys.AddLink(revolute)

vis = chronoirr.ChVisualSystemIrrlicht()                            # Irrlicht render window
vis.AttachSystem(sys)                                              # bind the physical system
vis.SetWindowSize(1280, 720)                                      # window resolution
vis.SetWindowTitle("Simple Pendulum")                            # window title
vis.Initialize()                                                  # create the device FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))  # PyChrono logo
vis.AddSkyBox()                                                   # sky box backdrop
vis.AddCamera(chrono.ChVector3d(0, 0, 4), chrono.ChVector3d(0.5, 0, 0))  # front view of the XY swing plane
vis.AddTypicalLights()                                            # standard lighting

time_step = 1e-3                                                  # integration step [s]
sim_end = 10.0                                                    # total simulated time [s]
render_fps = 50.0                                                 # target frames per second
render_every = max(1, round(1.0 / (render_fps * time_step)))     # untagged cadence constant
log_info = True                                                  # fire the physics log once
while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)                            # advance one step
        if log_info and sys.GetChTime() > 1:                     # after 1 s of settling
            pos = pendulum.GetPos()                              # position of the bob
            print("t = ", sys.GetChTime())
            print("     ", pos.x, "  ", pos.y)
            vel = pendulum.GetPosDt()                            # linear velocity
            print("     ", vel.x, "  ", vel.y)
            log_info = False                                     # disable further logging
        if sys.GetChTime() >= sim_end:
            break
