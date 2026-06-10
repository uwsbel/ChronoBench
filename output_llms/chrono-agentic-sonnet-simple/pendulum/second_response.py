import os
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

sys = chrono.ChSystemNSC()                                           # NSC system for rigid pendulum
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -1.62, 0))    # moon gravity (1.62 m/s^2)

# Ground body (fixed pivot at origin)
ground = chrono.ChBody()
ground.SetFixed(True)                                                # immovable anchor
ground.SetPos(chrono.ChVector3d(0, 0, 0))
sys.AddBody(ground)

# Sphere visual on the joint pivot with radius = 2
joint_sphere = chrono.ChVisualShapeSphere(2.0)                       # radius = 2 as specified
joint_sphere.SetColor(chrono.ChColor(0.7, 0.3, 0.3))               # red tint for visibility
ground.AddVisualShape(joint_sphere, chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))

# Pendulum body parameters
pend_mass = 2.0                                                      # mass = 2 kg
pend_length = 1.5                                                    # cylinder height = 1.5
pend_radius = 0.1                                                    # cylinder radius = 0.1

# Pendulum hangs downward; center at half-length below pivot
pend = chrono.ChBody()
pend.SetMass(pend_mass)
pend.SetInertiaXX(chrono.ChVector3d(0.4, 1.5, 1.5))                 # inertia tensor as specified
pend.SetPos(chrono.ChVector3d(0, -pend_length / 2, 0))              # center at y = -0.75
pend.EnableCollision(False)
sys.AddBody(pend)

# Cylinder visual along Y axis; default ChVisualShapeCylinder axis is Z, rotate Z->Y via AngleX
cyl_shape = chrono.ChVisualShapeCylinder(pend_radius, pend_length)   # r=0.1, h=1.5
cyl_shape.SetColor(chrono.ChColor(0.3, 0.6, 0.8))                  # blue for contrast
pend.AddVisualShape(cyl_shape, chrono.ChFramed(
    chrono.VNULL,
    chrono.QuatFromAngleX(-chrono.CH_PI_2)                           # rotate cylinder Z-axis to Y-axis
))

# Initial angular velocity for the pendulum (planar swing in XY, rotation about Z)
pend.SetAngVelParent(chrono.ChVector3d(0, 0, 2.0))                   # initial angular velocity 2 rad/s

# Spherical joint connecting pendulum top to ground pivot at world origin
# 3-arg form: ChFramed position is in WORLD space
joint = chrono.ChLinkLockSpherical()
joint.Initialize(pend, ground, chrono.ChFramed(
    chrono.ChVector3d(0, 0, 0),                                      # world-space pivot at origin
    chrono.QUNIT
))
sys.AddLink(joint)

# Irrlicht visualization — camera placed far enough to see the large sphere (r=2)
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)                    # Y-up
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Pendulum on Moon - Spherical Joint")
vis.Initialize()                                                     # Initialize FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 2, 20), chrono.ChVector3d(0, -1, 0))  # far camera to see sphere+arm
vis.AddTypicalLights()

time_step = 1e-3                                                     # integration step
sim_end = 10.0                                                       # simulation duration
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))        # untagged cadence constant
log_info = True                                                      # one-shot physics log flag
while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene(); vis.Render(); vis.EndScene()
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)
        if log_info and sys.GetChTime() > 1:                        # after 1 s of dynamics
            pos = pend.GetPos()                                     # pendulum center position
            print("t = ", sys.GetChTime())
            print("     ", pos.x, "  ", pos.y)
            vel = pend.GetPosDt()                                   # linear velocity
            print("     ", vel.x, "  ", vel.y)
            log_info = False                                        # disable further logging
        if sys.GetChTime() >= sim_end:
            break
