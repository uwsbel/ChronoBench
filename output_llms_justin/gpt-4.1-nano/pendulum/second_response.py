import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import math

# Initialize the Chrono simulation system
sys = chrono.ChSystemNSC()

# Set the gravitational acceleration for the system (on the Moon)
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -1.62, 0))

# Create the ground body and add it to the simulation
ground = chrono.ChBody()
sys.Add(ground)
ground.SetFixed(True)  # The ground is fixed and does not move
ground.EnableCollision(False)  # Disable collision detection for the ground

# Add a visualization cylinder to the ground
cyl_1 = chrono.ChVisualShapeCylinder(0.2, 0.4)  # Cylinder with radius 0.2 and height 0.4
ground.AddVisualShape(cyl_1, chrono.ChFramed(chrono.ChVector3d(0, 0, +1)))

# Create a pendulum body and add it to the simulation
pend_1 = chrono.ChBody()
sys.AddBody(pend_1)
pend_1.SetFixed(False)  # The pendulum can move
pend_1.EnableCollision(False)  # Disable collision detection for the pendulum

# Set mass and inertia tensor
pend_1.SetMass(2)  # Set the mass to 2 kg
pend_1.SetInertiaXX(chrono.ChVector3d(0.4, 1.5, 1.5))  # Updated inertia tensor

# Add a visualization cylinder with adjusted dimensions
cyl_1 = chrono.ChVisualShapeCylinder(0.1, 1.5)  # Radius 0.1, height 1.5
cyl_1.SetColor(chrono.ChColor(0.6, 0, 0))  # Set the color of the cylinder (RGB)
# Set the initial position so that the pendulum hangs from the pivot point
# To align properly, place it at its initial position
pend_1.AddVisualShape(cyl_1, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))

# Set the initial position of the pendulum (center of mass)
# Place it at (1, 0, 1) as before, but since the pendulum now has a length of 1.5,
# we can adjust if needed. For now, keep initial position the same.
pend_1.SetPos(chrono.ChVector3d(1, 0, 1))
# Also set an initial angular velocity
pend_1.SetW(chrono.ChVector3d(0, 0, 5))  # For example, initial angular velocity around Z

# Create a spherical joint to connect the pendulum to the ground
# Remove the revolute joint and replace with a ChLinkLockSpherical
joint_pos = chrono.ChVector3d(0, 0, 1)  # Pivot point position, same as previous revolute axis point
spherical_joint = chrono.ChLinkLockSpherical()
spherical_joint.Initialize(ground, pend_1, chrono.ChFrameD(joint_pos))
sys.AddLink(spherical_joint)

# Visualization of the joint as a sphere with radius 2
sphere_visual = chrono.ChVisualShapeSphere(2)
sphere_visual.SetColor(chrono.ChColor(0, 0, 1))  # Optional: make the joint sphere blue
# Attach the visual shape to the joint
# First, create a visualization shape for the joint (optional, since visualization of joints is not directly supported)
# As a workaround, add visual shape to the body at joint position
# For simplicity, attach to the pendulum body at the pivot point
pend_1.AddVisualShape(sphere_visual, chrono.ChFrameD(joint_pos))

# Set the initial angular velocity (already done above with SetW)

# Create the Irrlicht visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)  # Attach the Chrono system to the visualization
vis.SetWindowSize(1024, 768)  # Set the window size for the visualization
vis.SetWindowTitle('Single Pendulum on Moon')  # Changed title for clarity
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