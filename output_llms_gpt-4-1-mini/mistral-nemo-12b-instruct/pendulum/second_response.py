import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import math

# Initialize the Chrono simulation system
sys = chrono.ChSystemNSC()

# Set the gravitational acceleration for the system (in m/s^2) to simulate the pendulum on the moon
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -1.62, 0))  # g = 1.62 m/s^2 on the moon

# Create the ground body and add it to the simulation
ground = chrono.ChBody()
sys.Add(ground)
ground.SetFixed(True)  # The ground is fixed and does not move
ground.EnableCollision(False)  # Disable collision detection for the ground

# Add a visualization sphere to the ground (joint)
joint_vis = chrono.ChVisualShapeSphere(2)  # Sphere with radius 2
ground.AddVisualShape(joint_vis, chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))

# Create a pendulum body and add it to the simulation
pend_1 = chrono.ChBody()
sys.AddBody(pend_1)
pend_1.SetFixed(False)  # The pendulum can move
pend_1.EnableCollision(False)  # Disable collision detection for the pendulum
pend_1.SetMass(2)  # Set the mass of the pendulum (in kg)
pend_1.SetInertiaXX(chrono.ChVector3d(0.4, 1.5, 1.5))  # Set the inertia tensor (in kg·m^2)

# Add a visualization cylinder to the pendulum (modified dimensions)
cyl_1 = chrono.ChVisualShapeCylinder(0.1, 1.5)  # Cylinder with radius 0.1 and height 1.5
cyl_1.SetColor(chrono.ChColor(0.6, 0, 0))  # Set the color of the cylinder (RGB)
pend_1.AddVisualShape(cyl_1, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))

# Set the initial position of the pendulum (center of mass) in the absolute frame
# The pendulum is initially horizontal, pointing towards the positive X direction
pend_1.SetPos(chrono.ChVector3d(1, 0, 1))

# Set an initial angular velocity for the pendulum
pend_1.SetWvel_par(chrono.ChVector3d(0, 0, 0.5))  # Angular velocity around Z-axis (in rad/s)

# Create a spherical joint to connect the pendulum to the ground
# The spherical joint allows rotation around any axis
sph_1 = chrono.ChLinkLockSpherical()
sph_1.Initialize(ground, pend_1, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.ChQuaterniond(1, 0, 0, 0)))
sys.AddLink(sph_1)

# ... (The rest of the script remains unchanged)