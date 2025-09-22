import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np

# Create the simulation system
sys = chrono.ChSystemNSC()

# Create the ground body
ground = chrono.ChBody()
ground.SetBodyFixed(True)
sys.Add(ground)

# Create the pendulum body
pendulum = chrono.ChBody()
pendulum.SetMass(1.0)  # Mass in kg
pendulum.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))  # Inertia in kg*m^2
pendulum.SetPos(chrono.ChVectorD(0, -1, 0))  # Initial position
pendulum.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))  # Initial rotation (identity quaternion)
sys.Add(pendulum)

# Create a revolute joint between the ground and pendulum
rev_joint = chrono.ChLinkRevolute()
rev_joint.Initialize(ground, pendulum, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.Q_ROTATE_Z_TO_Y))
sys.AddLink(rev_joint)

# Initialize the Irrlicht visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle('Simple Pendulum Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(3, 3, -3))
vis.AddTypicalLights()

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.01)  # Step the simulation dynamics
    
    # Log the pendulum's position and velocity
    if sys.GetChTime() % 0.1 < 0.01:  # Log every 0.1 seconds
        pos = pendulum.GetPos()
        vel = pendulum.GetPos_dt()
        print(f"Time: {sys.GetChTime():.2f}, Position: ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f}), Velocity: ({vel.x:.2f}, {vel.y:.2f}, {vel.z:.2f})")

    # Handle user input to close the simulation
    if vis.GetDevice().isWindowActive():
        if vis.GetDevice().getEventReceiver().KeyIsPressed(irrlicht.EKEY_ESCAPE):
            break