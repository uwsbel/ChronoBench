import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math

# Initialize the PyChrono environment
chrono.SetChronoDataPath("/path/to/chrono/data/")  # Optional, if you have custom data path

# Create a physical system
sys = chrono.ChSystemNSC()

# Create a ground body
ground = chrono.ChBody()
ground.SetBodyFixed(True)
sys.Add(ground)

# Create a pendulum body
pendulum = chrono.ChBody()
pendulum.SetMass(1.0)
pendulum.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))
pendulum.SetPos(chrono.ChVectorD(0, -1, 0))
sys.Add(pendulum)

# Create a revolute joint between the ground and the pendulum
rev_joint = chrono.ChLinkRevolute()
rev_joint.Initialize(ground, pendulum, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.Q_from_AngY(math.pi/2)))
sys.Add(rev_joint)

# Create an Irrlicht application
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle('Simple Pendulum Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataPath() + 'logo_pychrono_alpha.png')
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 0, 3))
vis.AddTypicalLights()

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    # Log the pendulum's position and velocity periodically
    if sys.GetChTime() % 0.1 < 0.01:
        print(f"Time: {sys.GetChTime():.2f}, Pendulum Position: {pendulum.GetPos().x:.2f}, {pendulum.GetPos().y:.2f}, Pendulum Velocity: {pendulum.GetPos_dt().x:.2f}, {pendulum.GetPos_dt().y:.2f}")
    
    # Step the simulation
    sys.DoStepDynamics(0.01)

# Clean up is handled by Python's garbage collector



import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math

# Initialize the PyChrono environment
chrono.SetChronoDataPath("/path/to/chrono/data/")  # Optional, if you have custom data path

# Create a physical system
sys = chrono.ChSystemNSC()

# Create a ground body
ground = chrono.ChBody()
ground.SetBodyFixed(True)
sys.Add(ground)

# Create a pendulum body
pendulum = chrono.ChBody()
pendulum.SetMass(1.0)
pendulum.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))
pendulum.SetPos(chrono.ChVectorD(0, -1, 0))
sys.Add(pendulum)

# Create a revolute joint between the ground and the pendulum
rev_joint = chrono.ChLinkRevolute()
rev_joint.Initialize(ground, pendulum, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.Q_from_AngY(math.pi/2)))
sys.Add(rev_joint)

# Create an Irrlicht application
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle('Simple Pendulum Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataPath() + 'logo_pychrono_alpha.png')
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 0, 3))
vis.AddTypicalLights()

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    # Log the pendulum's position and velocity periodically
    if sys.GetChTime() % 0.1 < 0.01:
        print(f"Time: {sys.GetChTime():.2f}, Pendulum Position: {pendulum.GetPos().x:.2f}, {pendulum.GetPos().y:.2f}, Pendulum Velocity: {pendulum.GetPos_dt().x:.2f}, {pendulum.GetPos_dt().y:.2f}")
    
    # Step the simulation
    sys.DoStepDynamics(0.01)

# Clean up is handled by Python's garbage collector