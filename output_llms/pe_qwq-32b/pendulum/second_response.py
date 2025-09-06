import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import math

# Initialize the Chrono simulation system
sys = chrono.ChSystemNSC()
sys.SetGravity(chrono.ChVectorD(0, -1.62, 0))  # Moon gravity

# Create the ground body and add it to the simulation
ground = chrono.ChBody()
sys.Add(ground)
ground.SetBodyFixed(True)  # The ground is fixed and does not move
ground.SetCollide(False)  # Disable collision detection for the ground

# Add a visualization cylinder and sphere for the joint to the ground
ground_cyl = chrono.ChVisualShapeCylinder(0.2, 0.4)  # Original ground cylinder
ground.AddVisualShape(ground_cyl, chrono.ChFrameD(chrono.ChVectorD(0, 0, 1)))  # Position at (0,0,1)

# Add sphere visualization for the joint
joint_sphere = chrono.ChVisualShapeSphere(2)  # Radius 2
ground.AddVisualShape(joint_sphere, chrono.ChFrameD(chrono.ChVectorD(0, 0, 1)))  # Position at joint point

# Create a pendulum body and add it to the simulation
pend_1 = chrono.ChBody()
sys.Add(pend_1)
pend_1.SetBodyFixed(False)  # The pendulum can move
pend_1.SetCollide(False)  # Disable collision detection for the pendulum
pend_1.SetMass(2)  # New mass of 2 kg
pend_1.SetInertiaXX(chrono.ChVectorD(0.4, 1.5, 1.5))  # New inertia tensor

# Add a visualization cylinder to the pendulum with new dimensions
pend_cyl = chrono.ChVisualShapeCylinder(0.1, 1.5)  # Radius 0.1, height 1.5
pend_cyl.SetColor(chrono.ChColor(0.6, 0, 0))  # Red color
pend_1.AddVisualShape(pend_cyl, chrono.ChFrameD(chrono.VNULL, chrono.Q_from_AngAxis(chrono.CH_C_PI_2, chrono.ChVectorD.Y())))  # Rotate 90 degrees around Y-axis

# Set the initial position of the pendulum (center of mass)
pend_1.SetPos(chrono.ChVectorD(1, 0, 1))  # 1 meter along X from joint

# Set initial angular velocity (around Z-axis)
pend_1.SetWvel_loc(chrono.ChVectorD(0, 0, 1))  # 1 rad/s initial angular velocity

# Create a spherical joint to connect the pendulum to the ground
spher_1 = chrono.ChLinkLockSpherical()
spher_1.Initialize(ground, pend_1, chrono.ChFrameD(chrono.ChVectorD(0, 0, 1), chrono.QUNIT))  # Position at (0,0,1)
sys.AddLink(spher_1)

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
    sys.DoStepDynamics(1e-3)  # Time step of 1ms

    if log_info and sys.GetChTime() > 1:
        pos = pend_1.GetPos()
        print(f"t = {sys.GetChTime():.2f}")
        print(f"Position: ({pos.x:.2f}, {pos.y:.2f})")
        lin_vel = pend_1.GetPos_dt()
        print(f"Linear Velocity: ({lin_vel.x:.2f}, {lin_vel.y:.2f})")
        log_info = False