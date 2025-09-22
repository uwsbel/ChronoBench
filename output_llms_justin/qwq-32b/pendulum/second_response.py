import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import math

# Initialize the Chrono simulation system
sys = chrono.ChSystemNSC()

# Set gravitational acceleration to Moon's gravity (1.62 m/s²)
sys.SetGravitationalAcceleration(chrono.ChVectorD(0, -1.62, 0))

# Create the ground body
ground = chrono.ChBody()
sys.Add(ground)
ground.SetFixed(True)
ground.EnableCollision(False)

# Add ground cylinder visualization
cyl_ground = chrono.ChVisualShapeCylinder(0.2, 0.4)
ground.AddVisualShape(cyl_ground, chrono.ChFrameD(chrono.ChVectorD(0, 0, 1)))

# Add spherical joint visualization (radius 2)
sphere = chrono.ChVisualShapeSphere(2)
ground.AddVisualShape(sphere, chrono.ChFrameD(chrono.ChVectorD(0, 0, 1)))

# Create pendulum body
pend_1 = chrono.ChBody()
sys.AddBody(pend_1)
pend_1.SetFixed(False)
pend_1.EnableCollision(False)
pend_1.SetMass(2)  # 2 kg
pend_1.SetInertiaXX(chrono.ChVectorD(0.4, 1.5, 1.5))  # New inertia tensor

# Add pendulum cylinder visualization (radius 0.1, height 1.5)
cyl_pend = chrono.ChVisualShapeCylinder(0.1, 1.5)
cyl_pend.SetColor(chrono.ChColor(0.6, 0, 0))
pend_1.AddVisualShape(cyl_pend, chrono.ChFrameD(
    chrono.VNULL, chrono.Q_from_AngY(chrono.CH_C_PI_2)))

# Set initial position and angular velocity
pend_1.SetPos(chrono.ChVectorD(1, 0, 1))
pend_1.SetWvel_loc(chrono.ChVectorD(0, 0, 2))  # Initial angular velocity (rad/s)

# Create spherical joint
sph_1 = chrono.ChLinkLockSpherical()
sph_1.Initialize(ground, pend_1, chrono.ChFrameD(
    chrono.ChVectorD(0, 0, 1), chrono.QUNIT))
sys.AddLink(sph_1)

# Initialize visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Modified Spherical Pendulum Demo')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 3, 6))
vis.AddTypicalLights()

# Simulation loop
log_info = True
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(1e-3)

    if log_info and sys.GetChTime() > 1:
        pos = pend_1.GetPos()
        print(f"t = {sys.GetChTime():.2f}")
        print(f"Position: {pos.x:.2f}, {pos.y:.2f}")
        lin_vel = pend_1.GetPos_dt()
        print(f"Linear Velocity: {lin_vel.x:.2f}, {lin_vel.y:.2f}")
        log_info = False