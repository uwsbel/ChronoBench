import os
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

sys = chrono.ChSystemNSC()                                            # NSC system for rigid pendulum
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))      # Y-up gravity, g = 9.81

# --- ground (fixed reference body that carries the hinge) ---
ground = chrono.ChBody()                                              # fixed pivot body
ground.SetFixed(True)                                                 # immovable reference
sys.AddBody(ground)                                                   # register ground

pivot_pos = chrono.ChVector3d(0, 0, 0)                                # hinge location in world
pin = chrono.ChVisualShapeCylinder(0.04, 0.2)                         # small pin at the pivot
pin.SetColor(chrono.ChColor(0.3, 0.3, 0.3))                          # dark grey pin
ground.AddVisualShape(pin, chrono.ChFramed(pivot_pos, chrono.QUNIT))  # pin along world Z at pivot

# --- pendulum arm (manual ChBody so we can set mass/inertia and a link cylinder) ---
arm_len = 2.0                                                         # pendulum arm length [m]
arm_rad = 0.05                                                        # arm cylinder radius [m]
arm_mass = 1.0                                                        # pendulum mass [kg]
# arm modeled as a thin rod about its center; I = (1/12) m L^2 transverse, small about axis
Ixx = (1.0 / 12.0) * arm_mass * arm_len * arm_len                    # transverse inertia
Iyy = 0.5 * arm_mass * arm_rad * arm_rad                             # axial inertia (about rod axis)
pendulum = chrono.ChBody()                                            # the swinging arm
pendulum.SetMass(arm_mass)                                            # total arm mass
pendulum.SetInertiaXX(chrono.ChVector3d(Iyy, Ixx, Ixx))             # inertia tensor (rod along local X)
pendulum.SetPos(chrono.ChVector3d(arm_len / 2.0, 0, 0))             # COM at mid-arm, hanging from pivot
pendulum.SetRot(chrono.QUNIT)                                         # body-local X = world X (arm horizontal)
sys.AddBody(pendulum)                                                # register pendulum

arm_vis = chrono.ChVisualShapeCylinder(arm_rad, arm_len)             # cylinder spans the arm
arm_vis.SetColor(chrono.ChColor(0.2, 0.4, 0.8))                     # blue arm
# rotate cylinder default Z -> body-local X so it points along the arm
pendulum.AddVisualShape(arm_vis, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))
bob = chrono.ChVisualShapeSphere(0.12)                               # bob at the far end
bob.SetColor(chrono.ChColor(0.8, 0.2, 0.2))                        # red bob
pendulum.AddVisualShape(bob, chrono.ChFramed(chrono.ChVector3d(arm_len / 2.0, 0, 0)))  # far-end ball

# --- revolute joint: arm hinges to ground about world Z (planar XY swing) ---
hinge = chrono.ChLinkLockRevolute()                                  # single-DOF hinge
hinge.Initialize(pendulum, ground, chrono.ChFramed(pivot_pos, chrono.QUNIT))  # local +Z = world Z hinge
sys.AddLink(hinge)                                                   # register hinge

# --- Irrlicht visualization (Initialize first, then scene elements, NO grid) ---
vis = chronoirr.ChVisualSystemIrrlicht()                            # render window
vis.AttachSystem(sys)                                                # bind the physical system
vis.SetWindowSize(1280, 720)                                         # window pixels
vis.SetWindowTitle("Simple Pendulum")                              # title bar
vis.Initialize()                                                     # create device BEFORE scene nodes
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))   # pychrono logo
vis.AddSkyBox()                                                      # sky box
vis.AddCamera(chrono.ChVector3d(0, 0, 5), chrono.ChVector3d(1, 0, 0))  # face the XY swing plane
vis.AddTypicalLights()                                               # standard two-light setup

time_step = 1e-3                                                     # integration step [s]
sim_end = 10.0                                                       # total simulated time [s]
render_fps = 50.0                                                    # frames per second for review
render_every = max(1, round(1.0 / (render_fps * time_step)))        # physics steps per rendered frame
log_info = True                                                      # fire the physics log once
while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()                                                # start frame
    vis.Render()                                                    # draw scene
    vis.EndScene()                                                  # finish frame
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)                               # advance one step
        if log_info and sys.GetChTime() > 1:                        # after 1 s of settling
            pos = pendulum.GetPos()                                 # position of the bob/arm COM
            print("t = ", sys.GetChTime())
            print("     ", pos.x, "  ", pos.y)
            vel = pendulum.GetPosDt()                               # linear velocity
            print("     ", vel.x, "  ", vel.y)
            log_info = False                                        # disable further logging
        if sys.GetChTime() >= sim_end:
            break
