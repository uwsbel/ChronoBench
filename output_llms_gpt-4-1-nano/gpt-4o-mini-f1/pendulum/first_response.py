import pychrono as chrono
import pychrono.irrlicht as irr
import math

# Create the Chrono simulation system
sys = chrono.ChSystemNSC()

# Create the ground body
ground = chrono.ChBody()
sys.AddBody(ground)

# Create the pendulum body
pendulum = chrono.ChBody()
sys.AddBody(pendulum)

# Set the mass and inertia of the pendulum
pendulum.SetMass(1.0)
pendulum.SetInertiaXX(chrono.ChVector3d(0.1, 0.1, 0.1))

# Set the initial position and orientation of the pendulum
pendulum.SetPos(chrono.ChVector3d(1.0, 0, 0))
pendulum.SetRot(chrono.QuatFromAngleAxis(1.0, chrono.ChVector3d(0, 1, 0)))

# Create a revolute joint to connect the ground and the pendulum
joint = chrono.ChLinkLockRevolute()
joint.Initialize(ground, pendulum, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
sys.AddLink(joint)

# Create the Irrlicht visualization system
vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(chrono.ChVector2i(1024, 768))
vis.SetWindowTitle('Simple Pendulum Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 2, -5), chrono.ChVector3d(0, 1, 0))
vis.AddTypicalLights()
vis.AddLightDirectional()

# Set the simulation time step
time_step = 0.01

# Simulation loop
while vis.Run():
    # Render the scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Log the pendulum's position and velocity
    pos = pendulum.GetPos()
    vel = pendulum.GetPosDt()
    print(f"Time: {sys.GetChTime():.2f}, Pendulum Position: {pos}, Velocity: {vel}")

    # Advance the simulation by one time step
    sys.DoStepDynamics(time_step)