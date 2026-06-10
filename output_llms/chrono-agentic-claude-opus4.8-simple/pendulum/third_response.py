import os
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

sys = chrono.ChSystemNSC()                                            # NSC rigid-body system (pure jointed MBS)
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))      # g = 9.81 along -Y (Y-up)

# Ground: fixed body carrying the first hinge pin (axis along world Z at z=1)
ground = chrono.ChBody()                                              # fixed truss body
ground.SetFixed(True)                                                 # immovable
ground.EnableCollision(False)                                         # pure jointed MBS, no contact
pin = chrono.ChVisualShapeCylinder(0.2, 0.4)                          # short pin: radius 0.2, length 0.4
ground.AddVisualShape(pin, chrono.ChFramed(chrono.ChVector3d(0, 0, 1), chrono.QUNIT))  # pin at the hinge point
sys.Add(ground)

# First pendulum arm: rigid cylinder swinging on the ground revolute
pend_1 = chrono.ChBody()                                              # first arm
pend_1.SetMass(1)                                                     # mass 1 kg
pend_1.SetInertiaXX(chrono.ChVector3d(0.2, 1, 1))                     # diagonal inertia tensor
pend_1.SetPos(chrono.ChVector3d(1, 0, 1))                             # starts horizontal, center at x=1
pend_1.EnableCollision(False)                                         # no contact
cyl_1 = chrono.ChVisualShapeCylinder(0.2, 2)                          # radius 0.2, length 2
cyl_1.SetColor(chrono.ChColor(0.6, 0, 0))                             # dark red
pend_1.AddVisualShape(cyl_1, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))  # lay the cylinder along X
sys.Add(pend_1)

# Revolute joint ground--pend_1 at (0,0,1); hinge axis = world Z (QUNIT)
joint_1 = chrono.ChLinkLockRevolute()                                # first hinge
joint_1.Initialize(ground, pend_1, chrono.ChFramed(chrono.ChVector3d(0, 0, 1), chrono.QUNIT))
sys.AddLink(joint_1)

# Second pendulum arm: attached to the far end of the first arm, free to swing independently
pend_2 = chrono.ChBody()                                             # second arm
pend_2.SetMass(1)                                                    # mass 1 kg
pend_2.SetInertiaXX(chrono.ChVector3d(0.2, 1, 1))                    # diagonal inertia tensor
pend_2.SetPos(chrono.ChVector3d(3, 0, 1))                            # center at x=3, in line with the first arm
pend_2.EnableCollision(False)                                        # no contact
cyl_2 = chrono.ChVisualShapeCylinder(0.2, 2)                         # radius 0.2, length 2
cyl_2.SetColor(chrono.ChColor(0, 0, 0.6))                            # dark blue (distinguish the two arms)
pend_2.AddVisualShape(cyl_2, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))  # lay along X
sys.Add(pend_2)

# Revolute joint pend_1--pend_2 at (2,0,1); hinge axis = world Z (QUNIT) — second arm swings freely
joint_2 = chrono.ChLinkLockRevolute()                                # second hinge
joint_2.Initialize(pend_1, pend_2, chrono.ChFramed(chrono.ChVector3d(2, 0, 1), chrono.QUNIT))
sys.AddLink(joint_2)

# Irrlicht visualization (Initialize first, then add scene elements; NO grid)
vis = chronoirr.ChVisualSystemIrrlicht()                             # Irrlicht window
vis.AttachSystem(sys)                                                # bind the system
vis.SetWindowSize(1024, 768)                                         # window pixels
vis.SetWindowTitle("Double Pendulum")                                # title bar
vis.Initialize()                                                     # create device FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))     # logo
vis.AddSkyBox()                                                      # sky box
vis.AddCamera(chrono.ChVector3d(0, 3, 6))                            # eye position
vis.AddTypicalLights()                                              # standard two-light setup

time_step = 1e-3                                                     # integration step
render_fps = 50.0                                                   # review video frame rate
render_every = max(1, round(1.0 / (render_fps * time_step)))        # untagged cadence constant
log_info = True                                                    # fire the physics log once
while vis.Run():                                                   # scored core = plain truth loop
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)                              # advance one step
        if log_info and sys.GetChTime() > 1:                       # after 1 s of settling
            pos = pend_2.GetPos()                                  # position of the second arm
            print("t = ", sys.GetChTime())
            print("     ", pos.x, "  ", pos.y)
            vel = pend_2.GetPosDt()                                # linear velocity
            print("     ", vel.x, "  ", vel.y)
            log_info = False                                       # disable further logging
