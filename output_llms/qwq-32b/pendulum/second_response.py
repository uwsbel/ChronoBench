import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import math

# Initialize the Chrono simulation system
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVectorD(0, -1.62, 0))  # Moon's gravity

# Create the ground body and add it to the simulation
ground = chrono.ChBody()
sys.Add(ground)
ground.SetFixed(True)
ground.EnableCollision(False)

# Add a sphere visualization for the joint at the anchor point
sphere = chrono.ChVisualShapeSphere(2)  # Radius 2
ground.AddVisualShape(sphere, chrono.ChFrameD(chrono.ChVectorD(0, 0, 1)))

# Create a pendulum body and add it to the simulation
pend_1 = chrono.ChBody()
sys.AddBody(pend_1)
pend_1.SetFixed(False)
pend_1.EnableCollision(False)
pend_1.SetMass(2)  # Adjusted mass
pend_1.SetInertiaXX(chrono.ChVectorD(0.4, 1.5, 1.5))  # Adjusted inertia tensor

# Add a visualization cylinder to the pendulum with new dimensions
cyl_1 = chrono.ChVisualShapeCylinder(0.1, 1.5)  # Radius 0.1, height 1.5
cyl_1.SetColor(chrono.ChColor(0.6, 0, 0))
pend_1.AddVisualShape(cyl_1, chrono.ChFrameD(
    chrono.VNULL,
    chrono.ChQuaternionD.From_AngY(chrono.CH_C_PI_2)  # Rotate 90 degrees around Y-axis
))

# Set initial position and angular velocity
pend_1.SetPos(chrono.ChVectorD(1, 0, 1))
pend_1.SetWvel_par(chrono.ChVectorD(0, 0, 1))  # Initial angular velocity around Z-axis

# Create a spherical joint to connect the pendulum to the ground
sph_1 = chrono.ChLinkLockSpherical()
sph_1.Initialize(
    ground,
    pend_1,
    chrono.ChFrameD(chrono.ChVectorD(0, 0, 1), chrono.ChQuaternionD(1, 0, 0, 0))
)
sys.AddLink(sph_1)

# Create the Irrlicht visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Modified Pendulum Simulation')
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
    sys.DoStepDynamics(1e-3)  # Time step of 0.001 seconds

    # Log information after 1 second of simulation time
    if log_info and sys.GetChTime() > 1:
        pos = pend_1.GetPos()
        print(f"t = {sys.GetChTime():.2f}")
        print(f"Position: {pos.x:.2f}, {pos.y:.2f}")
        lin_vel = pend_1.GetPos_dt()
        print(f"Linear Velocity: {lin_vel.x:.2f}, {lin_vel.y:.2f}")
        log_info = False