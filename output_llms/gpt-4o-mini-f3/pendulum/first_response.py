import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import math

# Initialize the Chrono simulation system
sys = chrono.ChSystemNSC()

# Set the gravitational acceleration for the system (m/s^2)
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))  # g = 9.81 m/s^2

# Add a ground body to the simulation
ground = chrono.ChBody()
sys.Add(ground)
ground.SetFixed(True)  # The ground is fixed and does not move
ground.EnableCollision(False)  # Disable collision detection for the ground

# Add a visualization shape to the ground
ground.AddVisualShape(chrono.ChVisualShapeBox(1, 1, 1), chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))

# Create a pendulum body and add it to the simulation
pend Body = chrono.ChBody()
sys.AddBody(pend Body)
pend Body.SetFixed(False)  # The pendulum can move
pend Body.EnableCollision(False)  # Disable collision detection for the pendulum
pend Body.SetMass(1)  # Set the mass of the pendulum (kg)
pend Body.SetInertiaXX(chrono.ChVector3d(0.2, 1, 1))  # Set the inertia tensor (kg·m^2)

# Add a visualization shape to the pendulum
pend Body.AddVisualShape(chrono.ChVisualShapeBox(0.2, 1, 1), chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))

# Set the initial position of the pendulum (center of mass) in the absolute frame
# The pendulum is initially horizontal, pointing towards the positive X direction
pend Body.SetPos(chrono.ChVector3d(1, 0, 1))

# Create a revolute joint to connect the pendulum to the ground
# The revolute joint allows rotation around a single axis (the Z-axis in this case)
rev_joint = chrono.ChLinkLockRevolute()
rev_joint.Initialize(ground, pend Body, chrono.ChFramed(chrono.ChVector3d(0, 0, 1), chrono.ChQuaterniond(1, 0, 0, 0)))
sys.AddLink(rev_joint)

# Create the Irrlicht visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)  # Attach the Chrono system to the visualization
vis.SetWindowSize(1024, 768)  # Set the window size for the visualization
vis.SetWindowTitle('ChBodyAuxRef demo')  # Set the window title
vis.Initialize()  # Initialize the visualization system
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))  # Add a logo to the visualization
vis.AddSkyBox()  # Add a skybox for better visual appearance
vis.AddCamera(chrono.ChVector3d(0, 3, 6))  # Add a camera to the visualization
vis.AddTypicalLights()  # Add typical lights for better visualization

# Simulation loop
log_info = True  # Flag to control logging
while vis.Run():
    vis.BeginScene()  # Begin the scene for rendering
    vis.Render()  # Render the scene
    vis.EndScene()  # End the scene
    sys.DoStepDynamics(1e-3)  # Advance the simulation by a time step of 0.001 seconds

    # Log information after 1 second of simulation time
    if log_info and sys.GetChTime() > 1:
        pos = pend Body.GetPos()  # Get the position of the pendulum
        print("t = ", sys.GetChTime())
        print("     ", pos.x, "  ", pos.y)
        lin_vel = pend Body.GetPosDt()  # Get the linear velocity of the pendulum
        print("     ", lin_vel.x, "  ", lin_vel.y)
        log_info = False  # Disable further logging