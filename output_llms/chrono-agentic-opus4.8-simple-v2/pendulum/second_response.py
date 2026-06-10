import os
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

sys = chrono.ChSystemNSC()                                           # rigid-body multibody system
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -1.62, 0))     # lunar gravity (moon surface)

# truss / ground — the fixed body the pendulum hangs from
ground = chrono.ChBody()                                             # fixed reference body
ground.SetFixed(True)                                                # immovable truss
ground.SetName("ground")                                             # name the body
sys.AddBody(ground)                                                  # register the ground

# visualize the joint (pivot) location with a sphere of radius 2
joint_sphere = chrono.ChVisualShapeSphere(2)                         # joint marker, radius = 2
joint_sphere.SetColor(chrono.ChColor(0.2, 0.4, 0.8))                # blue joint marker
ground.AddVisualShape(joint_sphere, chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))  # at the pivot

# pendulum body — a horizontal rod whose near end pivots at the origin
pend = chrono.ChBody()                                               # the swinging pendulum
pend.SetMass(2)                                                      # mass = 2 kg
pend.SetInertiaXX(chrono.ChVector3d(0.4, 1.5, 1.5))                  # inertia tensor (Ixx, Iyy, Izz)
pend.SetPos(chrono.ChVector3d(0.75, 0, 0))                           # COM at half the rod length (1.5/2)
pend.SetAngVelParent(chrono.ChVector3d(0, 0, 2.0))                   # initial angular velocity about Z
sys.AddBody(pend)                                                    # register the pendulum

# visual cylinder for the pendulum rod: radius 0.1, height (length) 1.5, along body-local X
pend_cyl = chrono.ChVisualShapeCylinder(0.1, 1.5)                    # radius = 0.1, height = 1.5
pend_cyl.SetColor(chrono.ChColor(0.6, 0.2, 0.2))                    # red rod
pend.AddVisualShape(pend_cyl, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))  # Z->X

# spherical joint at the origin connecting the pendulum to the ground (replaces the revolute)
joint = chrono.ChLinkLockSpherical()                                # ball joint — 3 rotational DOF
joint.Initialize(pend, ground, chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))  # pivot at world origin
sys.AddLink(joint)                                                   # register the joint

# Irrlicht visualization (Initialize first, then scene elements; no grid)
vis = chronoirr.ChVisualSystemIrrlicht()                            # 3D render window
vis.AttachSystem(sys)                                                # bind the physical system
vis.SetWindowSize(1280, 720)                                         # window resolution
vis.SetWindowTitle("Single pendulum (lunar gravity, spherical joint)")  # window title
vis.Initialize()                                                     # create the device FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))    # PyChrono logo
vis.AddSkyBox()                                                      # sky background
vis.AddCamera(chrono.ChVector3d(2, 1, 14), chrono.ChVector3d(0.5, 0, 0))  # pulled back to clear the r=2 joint sphere
vis.AddTypicalLights()                                               # standard lighting

time_step = 1e-3                                                     # integration step [s]
sim_end = 10.0                                                       # total simulated time [s]
render_fps = 50.0                                                    # frames per second for review
render_every = max(1, round(1.0 / (render_fps * time_step)))        # untagged cadence constant
log_info = True                                                      # fire the physics log once
while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()                                                 # start the render frame
    vis.Render()                                                     # draw the scene
    vis.EndScene()                                                   # finish the render frame
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)                               # advance one physics step
        if log_info and sys.GetChTime() > 1:                        # log once after 1 s of swing
            pos = pend.GetPos()                                     # position of the pendulum COM
            print("t = ", sys.GetChTime())
            print("     ", pos.x, "  ", pos.y)
            vel = pend.GetPosDt()                                  # linear velocity of the COM
            print("     ", vel.x, "  ", vel.y)
            log_info = False                                        # disable further logging
        if sys.GetChTime() >= sim_end:
            break
