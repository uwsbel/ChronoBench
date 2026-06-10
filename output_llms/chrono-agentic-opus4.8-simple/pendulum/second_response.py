import os
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

sys = chrono.ChSystemNSC()                                            # rigid-body MBS system
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -1.62, 0))      # moon gravity (Y-up)

ground = chrono.ChBody()                                              # fixed ground / pivot anchor
ground.SetFixed(True)                                                 # immovable reference
sys.AddBody(ground)

joint_pos = chrono.ChVector3d(0, 0, 0)                                # pivot at the origin
joint_marker = chrono.ChVisualShapeSphere(2)                          # joint visual: sphere radius 2
joint_marker.SetOpacity(0.25)                                         # translucent so the arm stays visible
ground.AddVisualShape(joint_marker, chrono.ChFramed(joint_pos))       # mark the pivot point

pend_mass = 2.0                                                       # pendulum mass [kg]
pend_len = 1.5                                                        # pendulum length (cylinder height) [m]
pend = chrono.ChBody()                                                # the swinging pendulum body
pend.SetMass(pend_mass)                                               # 2 kg
pend.SetInertiaXX(chrono.ChVector3d(0.4, 1.5, 1.5))                   # inertia tensor
pend.SetPos(chrono.ChVector3d(0, -pend_len / 2, 0))                   # COM half a length below the pivot
sys.AddBody(pend)

pend_cyl = chrono.ChVisualShapeCylinder(0.1, pend_len)               # rod: radius 0.1, height 1.5
pend_cyl.SetColor(chrono.ChColor(0.8, 0.2, 0.2))                      # red rod for visibility
pend.AddVisualShape(pend_cyl, chrono.ChFramed(chrono.VNULL, chrono.QUNIT))  # default Z axis is vertical

pend.SetAngVelParent(chrono.ChVector3d(0, 0, 2.0))                    # initial angular velocity [rad/s]

joint = chrono.ChLinkLockSpherical()                                  # spherical joint at the pivot
joint.Initialize(pend, ground, chrono.ChFramed(joint_pos))           # connect pendulum to ground at origin
sys.AddLink(joint)

vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)                     # Y-up world
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Spherical Pendulum on the Moon")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(2, 0, 10), chrono.ChVector3d(0, -0.5, 0))
vis.AddTypicalLights()

time_step = 1e-3                                                      # integration step [s]
sim_end = 10.0                                                        # simulation duration [s]
render_fps = 50.0                                                     # frames per second for review
render_every = max(1, round(1.0 / (render_fps * time_step)))         # untagged cadence constant
log_info = True                                                       # fire the physics log once
while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)                                # advance one step
        if log_info and sys.GetChTime() > 1:                         # after 1 s of settling
            pos = pend.GetPos()                                      # position of the pendulum COM
            print("t = ", sys.GetChTime())
            print("     ", pos.x, "  ", pos.y)
            vel = pend.GetPosDt()                                    # linear velocity
            print("     ", vel.x, "  ", vel.y)
            log_info = False                                         # disable further logging
        if sys.GetChTime() >= sim_end:
            break
