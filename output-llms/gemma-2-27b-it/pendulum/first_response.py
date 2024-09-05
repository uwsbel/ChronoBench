import pychrono as chrono
import pychrono.irrlicht as chronoirr

# 1. Initialize the PyChrono environment and core components
my_system = chrono.ChSystemNSC()
my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  # Set gravity

# 2. Add the required physical systems and objects
# Create ground body
ground = chrono.ChBodyEasyBox(1, 1, 0.1, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, -0.5, 0))
my_system.Add(ground)

# Create pendulum bob
bob_radius = 0.2
bob_mass = 1
bob = chrono.ChBodyEasySphere(bob_radius, bob_mass, True, True)
bob.SetPos(chrono.ChVectorD(0, 1, 0))

# Create revolute joint
joint = chrono.ChLinkRevolute()
joint.Initialize(ground, bob, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0)))
my_system.Add(joint)

# 3. Set necessary default parameters
# Initial conditions (optional)
bob.SetPos_dt(chrono.ChVectorD(0, 0, 0))  # Initial velocity

# Visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(my_system)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle('Simple Pendulum')
vis.Initialize()
vis.AddTypicalSky()
vis.AddCamera(chrono.ChVectorD(2, 2, 2))
vis.SetCameraPosition(chrono.ChVectorD(2, 2, 2))

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    my_system.DoStepDynamics(0.01)  # Time step

    # Log pendulum position and velocity (example)
    print(f"Position: {bob.GetPos()}")
    print(f"Velocity: {bob.GetPos_dt()}")

    vis.EndScene()