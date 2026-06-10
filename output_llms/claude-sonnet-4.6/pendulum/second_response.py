import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import math

# Initialize the Chrono simulation system
sys = chrono.ChSystemNSC()

# Change gravitational acceleration to simulate moon gravity
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -1.62, 0))  # Moon gravity = 1.62 m/s^2

# Create the ground body and add it to the simulation
ground = chrono.ChBody()
sys.Add(ground)
ground.SetFixed(True)  # The ground is fixed and does not move
ground.EnableCollision(False)  # Disable collision detection for the ground

# Add a visualization cylinder to the ground (anchor point)
cyl_ground = chrono.ChVisualShapeCylinder(0.2, 0.4)  # Cylinder with radius 0.2 and height 0.4
ground.AddVisualShape(cyl_ground, chrono.ChFramed(chrono.ChVector3d(0, 0, 1)))

# Add a sphere visualization for the joint with radius 2
sphere_joint = chrono.ChVisualShapeSphere(2)  # Sphere with radius 2 to visualize the joint
ground.AddVisualShape(sphere_joint, chrono.ChFramed(chrono.ChVector3d(0, 0, 1)))

# Create a pendulum body and add it to the simulation
pend_1 = chrono.ChBody()
sys.AddBody(pend_1)
pend_1.SetFixed(False)  # The pendulum can move
pend_1.EnableCollision(False)  # Disable collision detection for the pendulum
pend_1.SetMass(2)  # Updated mass to 2 kg
pend_1.SetInertiaXX(chrono.ChVector3d(0.4, 1.5, 1.5))  # Updated inertia tensor (in kg·m^2)

# Add a visualization cylinder to the pendulum with updated dimensions (radius 0.1, height 1.5)
cyl_pend = chrono.ChVisualShapeCylinder(0.1, 1.5)  # Updated radius 0.1 and height 1.5
cyl_pend.SetColor(chrono.ChColor(0.6, 0, 0))  # Set the color of the cylinder (RGB)
pend_1.AddVisualShape(cyl_pend, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleY(chrono.CH_PI_2)))

# Set the initial position of the pendulum (center of mass) in the absolute frame
# With height 1.5, half-length = 0.75; joint is at (0, 0, 1), so center is at (0.75, 0, 1)
pend_1.SetPos(chrono.ChVector3d(0.75, 0, 1))

# Set an initial angular velocity for the pendulum (rotating around Z-axis)
pend_1.SetAngVelParent(chrono.ChVector3d(0, 0, 1.5))  # Initial angular velocity of 1.5 rad/s around Z

# Replace the revolute joint with a spherical joint
sph_1 = chrono.ChLinkLockSpherical()
sph_1.Initialize(ground, pend_1, chrono.ChFramed(chrono.ChVector3d(0, 0, 1), chrono.ChQuaterniond(1, 0, 0, 0)))
sys.AddLink(sph_1)

# Create the Irrlicht visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)  # Attach the Chrono system to the visualization
vis.SetWindowSize(1024, 768)  # Set the window size for the visualization
vis.SetWindowTitle('Pendulum on Moon with Spherical Joint')  # Set the window title
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
        print("     Position x = ", pos_1.x, "  y = ", pos_1.y, "  z = ", pos_1.z)
        lin_vel_1 = pend_1.GetPosDt()  # Get the linear velocity of the pendulum
        print("     Velocity x = ", lin_vel_1.x, "  y = ", lin_vel_1.y, "  z = ", lin_vel_1.z)
        ang_vel_1 = pend_1.GetAngVelParent()  # Get the angular velocity
        print("     Angular Velocity x = ", ang_vel_1.x, "  y = ", ang_vel_1.y, "  z = ", ang_vel_1.z)
        log_info = False  # Disable further logging