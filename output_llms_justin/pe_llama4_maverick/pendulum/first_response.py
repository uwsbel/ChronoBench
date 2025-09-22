import pychrono as chrono
import pychrono.irrlicht as chronoirr

# 1. Initialize the PyChrono environment and core components.
# Create a Chrono system
sys = chrono.ChSystemNSC()

# Set the gravitational acceleration
sys.Set_G_acc(chrono.ChVector3d(0, -9.81, 0))

# 2. Add the required physical systems and objects.
# Create the ground body
ground = chrono.ChBody()
ground.SetBodyFixed(True)
ground.SetPos(chrono.ChVector3d(0, 0, 0))
sys.Add(ground)

# Create the pendulum body
pendulum = chrono.ChBody()
pendulum.SetMass(1.0)  # Mass in kg
pendulum.SetInertiaXX(chrono.ChVector3d(0.1, 0.1, 0.1))  # Inertia tensor
pendulum.SetPos(chrono.ChVector3d(0, -1, 0))  # Initial position
sys.Add(pendulum)

# 3. Set necessary default parameters.
# Create a revolute joint between the ground and the pendulum
revolute_joint = chrono.ChLinkLockRevolute()
revolute_joint.Initialize(ground, pendulum, chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0)))
sys.Add(revolute_joint)

# Visualization setup
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle('Simple Pendulum Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(3, 3, -3))
vis.AddTypicalLights()

# Add visual shapes to the bodies
ground_shape = chrono.ChVisualShapeBox(10, 1, 10)
ground.AddVisualShape(ground_shape, chrono.ChFrameD(chrono.ChVector3d(0, -0.5, 0)))

pendulum_shape = chrono.ChVisualShapeSphere(0.2)
pendulum.AddVisualShape(pendulum_shape)

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    sys.DoStepDynamics(0.01)  # Step the simulation
    vis.EndScene()
    
    # Log the pendulum's position and velocity periodically
    if sys.GetChTime() % 1.0 < 0.01:
        print(f"Time: {sys.GetChTime():.2f}, Pendulum Position: {pendulum.GetPos()}, Velocity: {pendulum.GetPos_dt()}")