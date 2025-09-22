import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import math

# ------------------------------------------------------------------
# 1) Create the Chrono NSC system and set lunar gravity
# ------------------------------------------------------------------
sys = chrono.ChSystemNSC()
sys.Set_G_acc(chrono.ChVector3d(0, -1.62, 0))  
# ------------------------------------------------------------------
# 2) Create a fixed ground body
# ------------------------------------------------------------------
ground = chrono.ChBody()
ground.SetBodyFixed(True)
ground.SetCollide(False)
sys.Add(ground)

# ------------------------------------------------------------------
# 3) Visualize the joint as a sphere of radius 2 at the origin
# ------------------------------------------------------------------
joint_sphere = chrono.ChVisualShapeSphere(2.0)  
ground.AddVisualShape(
    joint_sphere,
    chrono.ChFrameD(chrono.ChVector3d(0, 0, 0))
)

# ------------------------------------------------------------------
# 4) Create the pendulum body
# ------------------------------------------------------------------
pendulum = chrono.ChBody()
pendulum.SetMass(2.0)
pendulum.SetInertiaXX(chrono.ChVector3d(0.4, 1.5, 1.5))
pendulum.SetCollide(False)
sys.Add(pendulum)

# Pendulum geometry parameters
pend_length = 1.5  # meters
pend_radius = 0.1  # meters

# Add a red cylinder to represent the pendulum rod.
# Default cylinder axis is Z; we rotate by +90° about Y so it points along +X
cyl_shape = chrono.ChVisualShapeCylinder(pend_radius, pend_length)
cyl_shape.SetColor(chrono.ChColor(0.6, 0.0, 0.0))
pendulum.AddVisualShape(
    cyl_shape,
    chrono.ChFrameD(
        chrono.ChVector3d(0, 0, 0),
        chrono.QuatFromAngleY(chrono.CH_PI_2)
    )
)

# Position the pendulum so that its COM sits at x = pend_length/2
pendulum.SetPos(chrono.ChVector3d(pend_length / 2.0, 0, 0))

# Give it an initial angular velocity (here about the Z‐axis)
pendulum.SetWvel_par(chrono.ChVector3d(0, 0, 1.0))

# ------------------------------------------------------------------
# 5) Connect ground & pendulum via a spherical joint at the origin
# ------------------------------------------------------------------
sph_joint = chrono.ChLinkLockSpherical()
# Initialize joint frame in absolute coords at (0,0,0)
sph_joint.Initialize(
    ground,
    pendulum,
    chrono.ChFrameD(chrono.ChVector3d(0, 0, 0))
)
sys.AddLink(sph_joint)

# ------------------------------------------------------------------
# 6) Set up Irrlicht visualization
# ------------------------------------------------------------------
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Spherical Joint Pendulum on the Moon')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 3, 6))
vis.AddTypicalLights()

# ------------------------------------------------------------------
# 7) Simulation loop
# ------------------------------------------------------------------
log_info = True
step_size = 1e-3

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    sys.DoStepDynamics(step_size)

    # After 1 second, print out COM position & velocity once
    if log_info and sys.GetChTime() > 1.0:
        pos = pendulum.GetPos()
        vel = pendulum.GetPos_dt()
        print(f"t = {sys.GetChTime():.3f} s")
        print(f"  pos = ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f})")
        print(f"  vel = ({vel.x:.3f}, {vel.y:.3f}, {vel.z:.3f})")
        log_info = False