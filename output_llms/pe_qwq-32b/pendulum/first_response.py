import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math

# Initialize the physical system
sys = chrono.ChSystemNSC()
sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  # Gravity vector

# Visualization setup
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Simple Pendulum Simulation")
vis.SetSymbolScale(0.01)
vis.SetShowInfos(True)
vis.SetCamera(chronoirr.ChCameraSettings(chrono.ChVectorD(0, 3, 5), chrono.ChVectorD(0, 0, 0)))
vis.SetCameraRotation(chrono.Q_from_Euler123(chrono.ChVectorD(0, math.pi/4, 0)))
vis.Initialize()
vis.AddSkyBox()
vis.AddTypicalLights()

# Ground body (fixed pivot point)
ground = chrono.ChBody()
ground.SetBodyFixed(True)
ground.SetPos(chrono.ChVectorD(0, 0, 0))
sys.Add(ground)

# Visual shape for the ground (small sphere at the pivot)
ground_shape = chrono.ChVisualShapeSphere(0.1)
ground.AddVisualShape(ground_shape)

# Pendulum parameters
pendulum_length = 2.0  # Length of the rod
pendulum_radius = 0.1  # Radius of the spherical bob
desired_mass = 1.0     # Desired mass of the pendulum

# Calculate density for the sphere to achieve the desired mass
volume = (4.0/3.0) * math.pi * (pendulum_radius)**3
density = desired_mass / volume

# Create the pendulum body (sphere)
pendulum = chrono.ChBodyEasySphere(pendulum_radius, density, True, True)
pendulum.SetPos(chrono.ChVectorD(0.5, -pendulum_length, 0))  # Initial displacement along X-axis
pendulum.SetMaterialSurface(chrono.ChMaterialSurface())  # Default material
sys.Add(pendulum)

# Revolute joint connecting ground and pendulum (rotation around Z-axis by default)
rev_joint = chrono.ChLinkLockRevolute()
rev_joint.Initialize(ground, pendulum, chrono.ChFrameD(chrono.ChVectorD(0,0,0), chrono.QUNIT))
sys.AddLink(rev_joint)

# Simulation loop
time_step = 0.001  # Time step for simulation
log_interval = 0.1  # Log every 0.1 seconds

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    sys.DoStepDynamics(time_step)
    
    current_time = sys.GetChTime()
    if current_time % log_interval < time_step:
        pos = pendulum.GetPos()
        vel = pendulum.GetPos_dt()
        print(f"Time: {current_time:.2f}s | Position: ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f}) | Velocity: ({vel.x:.2f}, {vel.y:.2f}, {vel.z:.2f})")

# Keep the window open until user closes it
vis.Run()