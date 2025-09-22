import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import math

# Initialize the Chrono simulation system
sys = chrono.ChSystemNSC()

# Set the gravitational acceleration for the system to simulate the moon gravity (in m/s^2)
sys.Set_G_acc(chrono.ChVector3d(0, -1.62, 0))  # Moon gravity

# Create the ground body and add it to the simulation
ground = chrono.ChBody()
sys.AddBody(ground)  # Use AddBody, not Add
ground.SetBodyFixed(True)  # Set it fixed; SetFixed is deprecated; use SetBodyFixed
ground.GetCollisionModel().ClearModel()  # Make sure collision model is cleared
ground.SetCollide(False)  # Disable collision detection for the ground

# Add a visualization cylinder to the ground
# Note: Chrono visualization shapes are normally created via ChCylinderShape or ChCylinderShapeEasy
# chrono.ChVisualShapeCylinder does not exist; the original code is incorrect
# Replace with proper visualization shape:

cyl_1 = chrono.ChCylinderShape()
cyl_1.GetCylinderGeometry().p1 = chrono.ChVector<>(0, 0, 0)  # Start point
cyl_1.GetCylinderGeometry().p2 = chrono.ChVector<>(0, 0, 0.4)  # End point (height 0.4)
cyl_1.GetCylinderGeometry().rad = 0.2  # Radius 0.2
ground.AddVisualShape(cyl_1)

# Alternatively, easier way:
# ground.AddVisualShape(chrono.ChCylinderShapeEasy(0.4, 0.2))

# But ChCylinderShapeEasy expects (height, radius)
ground.AddVisualShape(chrono.ChCylinderShapeEasy(0.4, 0.2))

# Create a pendulum body and add it to the simulation
pend_1 = chrono.ChBody()
sys.AddBody(pend_1)  # use AddBody, not Add
pend_1.SetBodyFixed(False)  # The pendulum moves
pend_1.SetCollide(False)  # Disable collision detection for the pendulum

# Adjusted mass and inertia tensor according to instructions
pend_1.SetMass(2)  # kg
pend_1.SetInertiaXX(chrono.ChVectorD(0.4, 1.5, 1.5))  # kg·m^2

# Add visualization cylinder with radius 0.1 and height 1.5
cyl_2 = chrono.ChCylinderShapeEasy(1.5, 0.1)
cyl_2.SetColor(chrono.ChColor(0.6, 0, 0))
# Orientation: in original code, it rotated cylinder 90 deg around Y (to be horizontal along X)
# We'll keep that
pend_1.AddVisualShape(cyl_2, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.ChQuaternionD().RotationY(chrono.CH_C_PI_2)))

# Set the initial position of pendulum - center of mass
# Adjust the position since the length changed to 1.5 (now pendulum length is shorter)
# The original pendulum was at (1,0,1), roughly horizontal in X direction
pend_1.SetPos(chrono.ChVectorD(0.75, 0, 1))  # half the length along X (since centered at COM)

# Set initial angular velocity for the pendulum
# We want to set some initial angular velocity, e.g., swinging in Y axis
# Angular velocity vector is in rad/s
# Let's set angular velocity around Z axis for a pendulum swinging in X-Y plane
pend_1.SetWvel_loc(chrono.ChVectorD(0, 0, 1))  # 1 rad/s around local Z axis

# Create a spherical joint to connect the pendulum to the ground
# The spherical joint allows rotation in 3 DOF at pivot point
# The pivot should be at pendulum top. Since pendulum center is at 0.75 (half length),
# the top end is at 1.5 along X from center, so at (0,0,1) + (-0.75,0,0) + (pivot)
# The pivot is at the joint location, which we put at (0,0,1)
pivot_pos = chrono.ChVectorD(0, 0, 1)  # same as original joint position

sph_joint = chrono.ChLinkLockSpherical()
# Initialize requires two bodies and the frame in absolute coords for the joint
sph_joint.Initialize(ground, pend_1, chrono.ChFrameD(pivot_pos))
sys.AddLink(sph_joint)

# Add visualization of the joint as a sphere radius 2 at pivot point
# Visualization attached to ground (so it doesn't move)
sphere_vis = chrono.ChSphereShape()
sphere_vis.SetSphereGeometry(chrono.ChVectorD(0, 0, 0), 2)  # sphere local center and radius
sphere_vis.SetColor(chrono.ChColor(0, 0, 1))  # blue color for visibility
# Attach visual shape with frame placing sphere at joint position
ground.AddVisualShape(sphere_vis, chrono.ChFrameD(pivot_pos))

# Create the Irrlicht visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)  # Attach the Chrono system to the visualization
vis.SetWindowSize(1024, 768)  # Set the window size for the visualization
vis.SetWindowTitle('Spherical Joint Single Pendulum on Moon Gravity')  # Updated title
vis.Initialize()  # Initialize the visualization system
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))  # Add a logo to the visualization
vis.AddSkyBox()  # Add a skybox for better visual appearance
vis.AddCamera(chrono.ChVectorD(0, 3, 6))  # Add a camera to the visualization (use ChVectorD for double)
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
        print("t = {:.3f}".format(sys.GetChTime()))
        print("     Position: x = {:.3f}, y = {:.3f}, z = {:.3f}".format(pos_1.x, pos_1.y, pos_1.z))
        lin_vel_1 = pend_1.GetPos_dt() if hasattr(pend_1, 'GetPos_dt') else pend_1.GetPosDt()  # Linear velocity method name varies
        # In PyChrono 6+, official binding is GetPos_dt()
        # GetPosDt() was used originally, probably older version.
        # We'll try GetPos_dt() first, then fallback to GetPosDt()
        if lin_vel_1 is None:
            lin_vel_1 = pend_1.GetPosDt()
        print("     Linear velocity: vx = {:.3f}, vy = {:.3f}, vz = {:.3f}".format(lin_vel_1.x, lin_vel_1.y, lin_vel_1.z))
        ang_vel_1 = pend_1.GetWvel_loc()
        print("     Angular velocity (local): wx = {:.3f}, wy = {:.3f}, wz = {:.3f}".format(ang_vel_1.x, ang_vel_1.y, ang_vel_1.z))
        log_info = False  # Disable further logging