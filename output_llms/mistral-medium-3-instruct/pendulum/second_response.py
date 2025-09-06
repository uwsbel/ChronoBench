import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import math

# Initialize the Chrono simulation system
sys = chrono.ChSystemNSC()

# Set the gravitational acceleration for the system (in m/s^2) - Moon gravity
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -1.62, 0))  # Moon gravity: 1.62 m/s^2

# Create the ground body and add it to the simulation
ground = chrono.ChBody()
sys.Add(ground)
ground.SetFixed(True)  # The ground is fixed and does not move
ground.EnableCollision(False)  # Disable collision detection for the ground

# Add a visualization sphere to represent the joint
sphere = chrono.ChVisualShapeSphere(0.02)  # Small sphere with radius 0.02 (2cm)
sphere.SetColor(chrono.ChColor(0, 0.6, 0))  # Green color for the joint
ground.AddVisualShape(sphere, chrono.ChFrameD(chrono.ChVector3d(0, 0, 1)))

# Create a pendulum body and add it to the simulation
pend_1 = chrono.ChBody()
sys.AddBody(pend_1)
pend_1.SetFixed(False)  # The pendulum can move
pend_1.EnableCollision(False)  # Disable collision detection for the pendulum
pend_1.SetMass(2)  # Set the mass of the pendulum to 2 kg
pend_1.SetInertiaXX(chrono.ChVector3d(0.4, 1.5, 1.5))  # Set the inertia tensor

# Add a visualization cylinder to the pendulum (modified dimensions)
cyl_1 = chrono.ChVisualShapeCylinder(0.1, 1.5)  # Cylinder with radius 0.1 and height 1.5
cyl_1.SetColor(chrono.ChColor(0.6, 0, 0))  # Set the color of the cylinder (RGB)
pend_1.AddVisualShape(cyl_1, chrono.ChFrameD(chrono.VNULL, chrono.Q_from_AngY(chrono.CH_PI_2)))

# Set the initial position of the pendulum (center of mass) in the absolute frame
pend_1.SetPos(chrono.ChVector3d(0, 0, 1 - 0.75))  # Adjusted for new cylinder height

# Set initial angular velocity (1 rad/s around Z axis)
pend_1.SetWvel_loc(chrono.ChVector3d(0, 0, 1))

# Create a spherical joint to connect the pendulum to the ground
sph_1 = chrono.ChLinkLockSpherical()
sph_1.Initialize(ground, pend_1, chrono.ChFrameD(chrono.ChVector3d(0, 0, 1)))
sys.AddLink(sph_1)

# Create the Irrlicht visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)  # Attach the Chrono system to the visualization
vis.SetWindowSize(1024, 768)  # Set the window size for the visualization
vis.SetWindowTitle('Modified Pendulum Simulation')  # Set the window title
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
        pos_1 = pend_1.GetPos()  # Get the position of the pendulum
        print("t = ", sys.GetChTime())
        print("     ", pos_1.x, "  ", pos_1.y)
        lin_vel_1 = pend_1.GetPosDt()  # Get the linear velocity of the pendulum
        print("     ", lin_vel_1.x, "  ", lin_vel_1.y)
        log_info = False  # Disable further logging