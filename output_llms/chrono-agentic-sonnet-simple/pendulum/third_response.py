import os
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# double pendulum: two arms linked in series, each pivoting freely
# Y-up world, gravity = (0, -9.81, 0); pure jointed MBS (no contact)

sys = chrono.ChSystemNSC()                                            # NSC for rigid MBS
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))     # g = 9.81 downward

# pendulum geometry
arm_length  = 1.0                                                     # each arm length [m]
arm_radius  = 0.05                                                    # visual cylinder radius [m]
arm_mass    = 1.0                                                     # each arm mass [kg]
arm_inertia = (1.0 / 12.0) * arm_mass * arm_length ** 2              # slender-rod Iz about COM [kg·m²]

# initial angles from vertical (both arms displaced to excite independent motion)
theta1 = math.radians(30)                                             # arm1 initial angle from vertical
theta2 = math.radians(-20)                                            # arm2 initial angle relative to arm1

# pivot of arm1 is fixed at origin
pivot1 = chrono.ChVector3d(0, 0, 0)

# arm1 COM in world: rotated theta1 from vertical
com1_x = math.sin(theta1) * arm_length / 2.0
com1_y = -math.cos(theta1) * arm_length / 2.0
com1 = chrono.ChVector3d(com1_x, com1_y, 0)

# tip of arm1 = pivot2 in world
tip1_x = math.sin(theta1) * arm_length
tip1_y = -math.cos(theta1) * arm_length
pivot2 = chrono.ChVector3d(tip1_x, tip1_y, 0)

# arm2 total angle from vertical
theta2_world = theta1 + theta2                                        # arm2 absolute angle
com2_x = tip1_x + math.sin(theta2_world) * arm_length / 2.0
com2_y = tip1_y - math.cos(theta2_world) * arm_length / 2.0
com2 = chrono.ChVector3d(com2_x, com2_y, 0)

# fixed support (ground anchor)
ground = chrono.ChBody()
ground.SetMass(1.0)
ground.SetInertiaXX(chrono.ChVector3d(1, 1, 1))
ground.SetPos(pivot1)
ground.SetFixed(True)
ground.EnableCollision(False)
vs_pin = chrono.ChVisualShapeSphere(0.06)                             # visual pivot pin
vs_pin.SetColor(chrono.ChColor(0.2, 0.2, 0.8))
ground.AddVisualShape(vs_pin)
sys.AddBody(ground)

# arm 1: cylinder, initially at angle theta1 from vertical
arm1 = chrono.ChBody()
arm1.SetMass(arm_mass)
arm1.SetInertiaXX(chrono.ChVector3d(arm_inertia, arm_inertia, arm_inertia))
arm1.SetPos(com1)
arm1.SetRot(chrono.QuatFromAngleZ(theta1))                            # tilt by theta1 in XY plane
arm1.EnableCollision(False)
cyl1 = chrono.ChVisualShapeCylinder(arm_radius, arm_length)           # cylinder along local Z by default
arm1.AddVisualShape(cyl1, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(chrono.CH_PI_2)))  # align local Y
vs_pin1 = chrono.ChVisualShapeSphere(0.055)                           # visual joint pin at arm1 far end
vs_pin1.SetColor(chrono.ChColor(0.8, 0.2, 0.2))
arm1.AddVisualShape(vs_pin1, chrono.ChFramed(chrono.ChVector3d(0, -arm_length / 2.0, 0)))
sys.AddBody(arm1)

# arm 2: cylinder, initially at angle theta2_world from vertical
arm2 = chrono.ChBody()
arm2.SetMass(arm_mass)
arm2.SetInertiaXX(chrono.ChVector3d(arm_inertia, arm_inertia, arm_inertia))
arm2.SetPos(com2)
arm2.SetRot(chrono.QuatFromAngleZ(theta2_world))                      # absolute world angle
arm2.EnableCollision(False)
cyl2 = chrono.ChVisualShapeCylinder(arm_radius, arm_length)
arm2.AddVisualShape(cyl2, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(chrono.CH_PI_2)))
sys.AddBody(arm2)

# hinge 1: ground ↔ arm1, pivot at origin; hinge axis = Z (swings in XY plane)
hinge1 = chrono.ChLinkLockRevolute()
hinge1.Initialize(
    ground, arm1, True,
    chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),       # on ground at origin
    chrono.ChFramed(chrono.ChVector3d(0, arm_length / 2.0, 0), chrono.QUNIT),  # top of arm1 (local)
)
sys.AddLink(hinge1)

# hinge 2: arm1 ↔ arm2, pivot at arm1's far end; hinge axis = Z
hinge2 = chrono.ChLinkLockRevolute()
hinge2.Initialize(
    arm1, arm2, True,
    chrono.ChFramed(chrono.ChVector3d(0, -arm_length / 2.0, 0), chrono.QUNIT),  # arm1 far end (local)
    chrono.ChFramed(chrono.ChVector3d(0,  arm_length / 2.0, 0), chrono.QUNIT),  # arm2 near end (local)
)
sys.AddLink(hinge2)

# Irrlicht visualization — Initialize() first, then scene elements
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)                     # Y-up scene
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Double Pendulum")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, -1.0, 4.0), chrono.ChVector3d(0, -1.0, 0))  # looking at pendulum midpoint
vis.AddTypicalLights()

time_step  = 1e-3                                                     # physics step [s]
sim_end    = 10.0                                                     # simulation end [s]
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))          # cadence constant (untagged)

log_info = True                                                        # one-shot physics log flag
while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene(); vis.Render(); vis.EndScene()
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)
        if log_info and sys.GetChTime() > 1.0:                        # log once after 1 s settling
            p1 = arm1.GetPos()
            print("t = ", sys.GetChTime())
            print("     ", p1.x, "  ", p1.y)
            v1 = arm1.GetPosDt()
            print("     ", v1.x, "  ", v1.y)
            log_info = False
        if sys.GetChTime() >= sim_end:
            break
