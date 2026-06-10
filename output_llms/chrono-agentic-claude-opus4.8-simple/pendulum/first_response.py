import os
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

sys = chrono.ChSystemNSC()                                           # rigid-body MBS system
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))     # Y-up gravity, g = 9.81

# ground: fixed body carrying the hinge support cylinder
ground = chrono.ChBody()                                             # the truss the pendulum hangs from
ground.SetFixed(True)                                                # immovable
ground.EnableCollision(False)                                        # pure jointed MBS, no contact
cyl_g = chrono.ChVisualShapeCylinder(0.2, 0.4)                       # support stub, radius 0.2, length 0.4
ground.AddVisualShape(cyl_g, chrono.ChFramed(chrono.ChVector3d(0, 0, 1), chrono.QUNIT))  # at hinge location
sys.Add(ground)

# pendulum: single rigid cylinder, starts horizontal pointing +X
pend = chrono.ChBody()                                               # the swinging arm
pend.SetMass(1)                                                      # mass = 1 kg
pend.SetInertiaXX(chrono.ChVector3d(0.2, 1, 1))                      # principal inertia of the rod
pend.SetPos(chrono.ChVector3d(1, 0, 1))                              # COM at +1 in X, in the z=1 plane
pend.EnableCollision(False)                                          # no contact
cyl_p = chrono.ChVisualShapeCylinder(0.2, 2)                         # arm: radius 0.2, length 2
cyl_p.SetColor(chrono.ChColor(0.6, 0, 0))                            # dark red
pend.AddVisualShape(cyl_p, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))  # lay along X
sys.Add(pend)

# revolute joint: hinge axis = world Z at the support (z=1 plane)
rev = chrono.ChLinkLockRevolute()                                    # one rotational DOF about +Z
rev.Initialize(ground, pend, chrono.ChFramed(chrono.ChVector3d(0, 0, 1), chrono.QUNIT))  # +Z hinge
sys.AddLink(rev)

# Irrlicht visualization (Initialize first, then scene elements, no grid)
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Simple Pendulum")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 3, 6))                            # eye above and behind
vis.AddTypicalLights()

time_step = 1e-3                                                     # integration step
render_fps = 50.0                                                    # review-video frame rate
render_every = max(1, round(1.0 / (render_fps * time_step)))         # untagged cadence constant
log_info = True                                                      # fire the physics log once
while vis.Run():                                                     # scored core = plain truth form
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)                                # advance one step
        if log_info and sys.GetChTime() > 1:                         # after 1 s of settling
            pos = pend.GetPos()                                      # position of the pendulum
            print("t = ", sys.GetChTime())
            print("     ", pos.x, "  ", pos.y)
            vel = pend.GetPosDt()                                    # linear velocity
            print("     ", vel.x, "  ", vel.y)
            log_info = False                                         # disable further logging
